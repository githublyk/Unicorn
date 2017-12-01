function plotCylinderLiveTest
%Plots the Snake in real time
%You need an active hebi linkage, and you might have to change the module
%number

    g = getSeaSnakeGroup();

    plt = CylinderPlotter('frame','gravity');

    while(true)
        plt.plot(g.getNextFeedback);
    end

end