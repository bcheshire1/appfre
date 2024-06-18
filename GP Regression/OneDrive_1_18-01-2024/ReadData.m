UnexploredDat = readmatrix("incomplete_map_array.txt");
UnexploredDat = reshape(UnexploredDat,238,109);
UnexploredDat = changem(UnexploredDat,[.4,0.2,0.9],[-1,0,100]);
CostmapLength = 238;
CostmapHeight = 109;
UnexploredValue = 0.4;
OpenValue = 0.2;
FrontierValue = 0.3;
for i = 1:CostmapLength
    for j = 1:CostmapHeight
        if (UnexploredDat(i,j) == UnexploredValue)%searches UnexploredDat for un
            if (i ~= 1)
                if (UnexploredDat((i-1),(j)) == OpenValue)
                    UnexploredDat((i-1),j) = FrontierValue;
                end
            end
            if (j ~= 1)
                if (UnexploredDat((i),(j-1)) == OpenValue)
                    UnexploredDat((i),(j-1)) = FrontierValue;
                end
            end
            if (i ~= 238)
                if (UnexploredDat((i+1),(j)) == OpenValue)
                    UnexploredDat((i+1),(j)) = FrontierValue;
                end
            end
            if (j ~= 109)
                if (UnexploredDat((i),(j+1)) == OpenValue)
                    UnexploredDat((i),(j+1)) = FrontierValue;
                end
            end
        end
    end
end

% UnexploredDat = readmatrix("incomplete_map_array.txt");
% UnexploredDat = reshape(UnexploredDat,238,109);
% UnexploredDat = changem(UnexploredDat,[.4,0.2,0.9],[-1,0,100]);
UnexploredCostmap = vehicleCostmap(UnexploredDat,"FreeThreshold",0.25,"OccupiedThreshold",0.65,"CellSize",0.65);
plot(UnexploredCostmap)