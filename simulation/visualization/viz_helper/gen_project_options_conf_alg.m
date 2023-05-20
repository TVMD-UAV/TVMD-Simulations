function options = gen_project_options_conf_alg(projectpath, projectname, filename, conf_id, alg_id)
    conf_map = ["model_A1_con", "model_A2_con", "model_A3_con", "model_A4_con", "model_A5_con", "model_A6_con", "model_A7_con", "model_A8_con", "model_A9_con", "model_A10_con", ...
                "model_A1_inc", "model_A2_inc", "model_A3_inc", "model_A4_inc", "model_A5_inc", "model_A6_inc", "model_A7_inc", "model_A8_inc", "model_A9_inc", "model_A10_inc", ...
                "model_T10_con", "model_T10_inc"];
    alg_map = ["moore", "nullspace", "redistributed", "null_redistributed"];
    conf_name = conf_map(conf_id);
    alg_name = alg_map(alg_id);

    check_path_exist(projectpath);
    outputpath = strcat(projectpath, "\\", projectname, "\\");
    check_path_exist(outputpath);
    outputpath = strcat(outputpath, "\\", conf_name, "\\");
    check_path_exist(outputpath);
    outputpath = strcat(outputpath, "\\", alg_name, "\\");
    check_path_exist(outputpath);
    
    key = {'projectpath', 'foldername', 'filename', 'lineStyle', 'markerStyle', 'savefig', ...
        'title_font_size', 'sgtitle_font_size', 'label_font_size', 'legend_font_size', ...
        'labely_pos'};
    value = {projectpath, outputpath, filename, '--', 'none', savefig, ...
        12, 14, 12, 10, ...
        -2.5};
    options = containers.Map(key, value);
end
