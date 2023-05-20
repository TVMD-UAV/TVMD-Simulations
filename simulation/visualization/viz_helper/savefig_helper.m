function savefig_helper(options, name)
    if options('savefig')
        fname = strcat(options('foldername'), options('filename'), name);
        saveas(gcf, strcat(fname, '.fig'));
        saveas(gcf, strcat(fname, '.svg'));
        saveas(gcf, strcat(fname, '.epsc'));
        movefile(strcat(fname, '.epsc'), strcat(fname, '.eps'))
    end
end
