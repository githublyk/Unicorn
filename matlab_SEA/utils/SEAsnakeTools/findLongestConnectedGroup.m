function gLongest = findLongestConnectedGroup
%FINDLONGESTCONNECTEDGROUP return the HebiGroup with the most
%connected modules. If more than one group is tied for longest the
%group with the alphabetically first module is returned.
    
    try
        gAll = HebiLookup.newGroupFromFamily('*');
    catch
        HebiLookup.clearModuleList;
        var = HebiLookup;
        pause(1.0) %HACK - I'd rather a true check to see if
                   %HebiLookup finished
        gAll = HebiLookup.newGroupFromFamily('*');
    end
    names = gAll.getInfo.name(1:gAll.getNumModules);
    
    longestGroupLength = 0;
    gLongest = [];
    
    while(numel(names) > longestGroupLength)
        gTemp = HebiLookup.newConnectedGroupFromName('*', names{1});
        names = setdiff(names, gTemp.getInfo.name);
        if(gTemp.getNumModules > longestGroupLength)
            longestGroupLength = gTemp.getNumModules;
            gLongest = gTemp;
        end
    end
end

