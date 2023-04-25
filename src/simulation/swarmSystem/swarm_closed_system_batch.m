alg_ids = [1 2 3]
for conf_id=16:16
    for alg_id=2:2
        clear swarm_closed_system_simulation
        swarm_closed_system_simulation(conf_id, alg_ids(alg_id));
    end
end
return

addpath('../controlAllocation/system_func')
conf_map = ["model_A3_con", "model_A4_con", "model_A5_con", "model_A6_con", "model_A7_con", "model_A8_con", "model_A9_con", "model_A10_con", ...
            "model_A3_inc", "model_A4_inc", "model_A5_inc", "model_A6_inc", "model_A7_inc", "model_A8_inc", "model_A9_inc", "model_A10_inc", ...
            "model_T10_con", "model_T10_inc"];
conf_legends = ["A3-Con", "A4-Con", "A5-Con", "A6-Con", "A7-Con", "A8-Con", "A9-Con", "A10-Con", ...
                "A3-Inc", "A4-Inc", "A5-Inc", "A6-Inc", "A7-Inc", "A8-Inc", "A9-Inc", "A10-Inc", ...
                "T10-Con", "T10-Inc"];
close all
figure('Position', [10 10 1200 350])
for conf_id=1:16
    [key, params] = get_swarm_params(conf_map(conf_id));

    subplot(2, 8, conf_id)
    n = length(params('psi'));
    f0 = reshape(get_f(zeros([n 1]), zeros([n 1]), ones([n 1])), [3 n]);
    plot_3kg_swarm(params, [0 0 0], eye(3), 128, f0); hold on
    % scatter3([-1 -1 1 1], [1 -1 1 -1], [0 0 0 0], 'MarkerFaceAlpha',0,'MarkerEdgeAlpha',0); hold on
    plot3([-1 -1 1 1], [1 -1 1 -1], [0 0 0 0], 'Color','none'); hold on
    xlim([-1 1])
    ylim([-1.5 1.5])
    zlim([-0.5 1])
    xlabel('x', 'FontName', 'Times New Roman', 'FontSize', 10)
    ylabel('y', 'FontName', 'Times New Roman', 'FontSize', 10)
    zlabel('z', 'FontName', 'Times New Roman', 'FontSize', 10)
    title(strcat("Model-", conf_legends(conf_id)), 'FontName', 'Times New Roman', 'FontSize', 12)
    axis equal
    grid on
    campos([3 5 5])
end

set(gcf, 'Renderer', 'painters')
print(gcf, '-depsc', 'test.eps')