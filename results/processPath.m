function path = processPath(filename, states_in_path)
%Used to visualize and characterize ompl path result txt file
% Griffin Van Anne

% open file
pathFile = fopen(filename);

formatSpec = '%s %c';
for i = 1:states_in_path
    formatSpec = append(formatSpec, ' %f');
end
formatSpec = append(formatSpec, ' %c');

% process
C = textscan(pathFile, formatSpec);

for i = 1:states_in_path
    path(:,i) = C{i+2};
end

end

