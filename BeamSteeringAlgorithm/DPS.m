function voltage = DPS(target_angle, null_angle)

    % Read measured phase set
    phase_set = load('phase_set.mat');
    set = phase_set.phase_set;

    % Global Parameters
    nt = 4; % number of Transmitters
    dt = 0.5; % Transmitter spacing / wavelength
    K = 4; % how many sets of solutions to compare

    theta = -90:1:90;
    st = exp(-1i*2*pi*dt*(0:nt-1).'*sind(theta)); % steering matrix

    % Beamformer
    A = st(:, null_angle + 91);
    P_A = eye(nt) - A / (A' * A) * A';
    w = P_A * st(:, target_angle + 91);

    [~, m] = max(abs(w));

    num = size(set,1);
    S = zeros(nt,num^2);
    S_ang = S;
    ang_ind = S;
    for i = 1:nt
        temp = set(:,2*i-1)+set(:,2*i).';
        S(i,:) = temp(:).';
        S_ang(i,:) = angle(S(i,:));
        [S_ang(i,:), ang_ind(i,:)] = sort(S_ang(i,:));
    end


    resi = num^2 / K;
    ind = zeros(K, nt);
    for k = 1:K
        Smk = S(m, ang_ind(m, (k-1)*resi+1: k*resi));
        [~, max_ind] = max(abs(Smk));
        alpha = Smk(max_ind) / w(m);
        for n = 1:nt
            if m == n
                ind(k,n) = ang_ind(m, max_ind + (k-1)*resi);
                continue
            end
            temp = w(n) * alpha;
            for j = 1:K
                Snk_min = S(n, ang_ind(n, (j-1)*resi+1));
                Snk_max = S(n, ang_ind(n, j*resi));
                if angle(temp) >= angle(Snk_min) && angle(temp) <= angle(Snk_max)
                    break
                end
            end
            [~, n_ind] = min(abs(S(n, ang_ind(n, (j-1)*resi+1: j*resi)) - temp));
            ind(k,n) = ang_ind(n, n_ind + (j-1)*resi);
        end
    end


    % Choose the best solution
    % w.r.t. null depth
    null_depth = zeros(K,1);
    st = exp(-1i*2*pi*0.5*(0:nt-1).'*sind([target_angle,null_angle]));

    for z = 1:K
        w_appro = zeros(nt,1);
        for zz = 1:nt
            w_appro(zz) = S(zz,ind(z,zz));
        end
        bp = (abs(w_appro'*st)).^2;
        bp = 10*log10(bp);
        bp = bp-max(bp);
        null_depth(z) = mean(bp(1) - bp(2:end));
    end

    [~, final_k] = max(null_depth);

    ind2 = ceil(ind(final_k,:).' / 76);
    ind1 = ind(final_k,:).' - (ind2 - 1) * 76;

    ind1 = transpose(ind1);
    ind2 = transpose(ind2);
    v1 = (ind1 - 1) * 0.2;
    v2 = (ind2 - 1) * 0.2;
    voltage = reshape([v1; v2], 1, []);
end

