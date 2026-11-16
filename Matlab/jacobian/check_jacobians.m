function check_jacobians(include_symbolic)
% Validate PoE Jacobians against differentiated DH FK and symbolic identities.
if nargin == 0
    include_symbolic = true;
end
g = robot_dimensions();
custom = g;
names = fieldnames(g);
for i = 1:numel(names)
    custom.(names{i}) = g.(names{i}) * (0.7 + 0.1*i);
end
geometries = {g,custom};
stream = RandStream('mt19937ar','Seed',42);
Q = [zeros(6,1), (pi/2)*eye(6), -(pi/2)*eye(6), ...
    [20;-35;50;15;-40;60]*pi/180, -pi+2*pi*rand(stream,6,100)];
max_fd = 0;

for geometry = 1:numel(geometries)
    dh = dh_data(geometries{geometry});
    poe = poe_data(geometries{geometry});
    for sample = 1:size(Q,2)
        q = Q(:,sample);
        [~,~,~,T0] = dh_fk(q,dh);
        for point = [5,6]
            % Differentiate the independent DH forward model in base axes.
            Jfd = finite_difference(q,point,dh);
            for frame = 0:6
                % Recover physical link axes from DH frame axes and their
                % constant home alignment, without using PoE FK for this check.
                C = poe.R_home(:,:,frame+1).'*dh.R_home(:,:,frame+1);
                R = T0(1:3,1:3,frame+1)*C.';
                J = poe_jacobian(q,frame,point,poe);
                max_fd = max(max_fd,norm(J-blkdiag(R.',R.')*Jfd,'fro'));
            end
        end
    end
    q = Q(:,14);
    for point = [5,6]
        assert(isequal(poe_jacobian(q,0,point,poe),poe_jacobian(q.',0,point,poe)));
    end
    J = poe_jacobian(q,3,5,poe);
    assert(norm(J(1:3,4:6),'fro') < 1e-10);
    assert(abs(norm(J(4:6,6))-1) < 1e-10);

    % Check the closed-form PoE joint motions against matrix exponentials.
    F = eye(4);
    for i = 1:6
        F = F * expm(poe.xi_hat(:,:,i)*q(i));
    end
    assert(norm(poe_fk(q,poe)-F*poe.M,'fro') < 1e-10);
end
assert(max_fd < 1e-7,'PoE Jacobian disagrees with differentiated DH forward kinematics.');
fprintf('PoE Jacobian vs finite differences of DH FK: %.3e.\n',max_fd);
fprintf('%d configurations, two geometries, points 5/6, frames 0:6.\n',size(Q,2));

if include_symbolic
    check_symbolic(geometries,Q(:,[1,14,15]));
end
end

function J = finite_difference(q,point,dh)
h = 1e-6;
T = dh_fk(q,dh);
J = zeros(6,6);
for i = 1:6
    delta = zeros(6,1);
    delta(i) = h;
    [Tp,~,~,Pp] = dh_fk(q+delta,dh);
    [Tm,~,~,Pm] = dh_fk(q-delta,dh);
    J(1:3,i) = (Pp(1:3,4,point+1)-Pm(1:3,4,point+1))/(2*h);
    W = ((Tp(1:3,1:3)-Tm(1:3,1:3))/(2*h))*T(1:3,1:3).';
    J(4:6,i) = [W(3,2)-W(2,3); W(1,3)-W(3,1); W(2,1)-W(1,2)]/2;
end
end

function check_symbolic(geometries,Q)
gs = robot_dimensions(true);
qs = sym('q',[6,1],'real');
fields = struct2cell(gs);
parameters = vertcat(fields{:});
poe = poe_data(gs);
syms a1 a2 d1 d2 d3 d4 d6 real
cases = [0,6; 0,5; 3,5];
max_error = 0;
for c = 1:size(cases,1)
    frame = cases(c,1);
    point = cases(c,2);
    Jpoe = simplify(poe_jacobian(qs,frame,point,poe));
    Jpoe_ad = dh_parameter_form(Jpoe);
    if frame == 3 && point == 5
        B = [1,0,cos(qs(5)); 0,-sin(qs(4)),cos(qs(4))*sin(qs(5)); ...
             0,cos(qs(4)),sin(qs(4))*sin(qs(5))];
        assert(isequal(simplify(Jpoe(1:3,4:6)),sym(zeros(3))));
        assert(isequal(simplify(expand(Jpoe(4:6,4:6)-B)),sym(zeros(3))));
        assert(isequal(simplify(expand(det(Jpoe(4:6,4:6))+sin(qs(5)))),sym(0)));
        % Check the report's arm determinant in its a/d notation.
        F1 = a2+d4*cos(qs(3));
        X = F1*cos(qs(2))+d3*sin(qs(2));
        D = -F1*sin(qs(2))+d3*cos(qs(2));
        arm_det = -d4/2*(a1*D*cos(qs(3))-sqrt(sym(3))*a1*F1*sin(qs(3)) ...
            -sqrt(sym(3))*X*(a2*sin(qs(3))-d2*cos(qs(3))));
        assert(isequal(simplify(expand(det(Jpoe_ad(1:3,1:3))-arm_det)),sym(0)), ...
            'Symbolic arm determinant disagrees with the report.');
    end
    evaluate_ad = matlabFunction(Jpoe_ad,'Vars',{qs,[a1;a2;d1;d2;d3;d4;d6]});
    evaluate_poe = matlabFunction(Jpoe,'Vars',{qs,parameters});
    for geometry = 1:numel(geometries)
        values = cell2mat(struct2cell(geometries{geometry}));
        dh_num = dh_data(geometries{geometry});
        poe_num = poe_data(geometries{geometry});
        ad_values = [dh_num.a(1:2),dh_num.d([1:4,6])].';
        for sample = 1:size(Q,2)
            q = Q(:,sample);
            J = poe_jacobian(q,frame,point,poe_num);
            max_error = max([max_error, ...
                norm(evaluate_ad(q,ad_values)-J,'fro'), ...
                norm(evaluate_poe(q,values)-J,'fro')]);
        end
    end
    fprintf('Symbolic PoE and a/d parameter forms checked: frame %d, point %d.\n',frame,point);
end
assert(max_error < 1e-10,'Symbolic evaluation disagrees with numerical PoE Jacobians.');
fprintf('Symbolic vs numerical PoE Jacobians: %.3e.\n',max_error);
fprintf('Exact wrist block, wrist determinant and report arm determinant agree.\n');
end
