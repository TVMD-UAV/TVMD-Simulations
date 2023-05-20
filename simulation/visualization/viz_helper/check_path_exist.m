function check_path_exist(path)
    if not(isfolder(path))
        mkdir(path);
    end
end
