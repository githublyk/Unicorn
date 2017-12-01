function g = getSeaSnakeGroup
%GETSEASNAKEGROUP returns the longest Hebi group if it contains
%more than 10 members, and prints an error otherwise. It is likely
%that this group will be the SEA snake.
    % HebiLookup.clearModuleList;
    % HebiLookup

    g = findLongestConnectedGroup();
    
    if(g.getNumModules < 10)
        disp('Longest connected group only has %f modules', g.getNumModules)
        error('No Snake Found on network')
    end
end
