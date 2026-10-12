function [figHandles] = save_all_figures(save_path)
%SAVE_ALL_FIGURES Summary of this function goes here
%   Detailed explanation goes here
% Get all open figure handles
figHandles = findobj('Type', 'figure');
save_path = strcat(save_path,'figures/');
if ~exist(save_path,'dir')
    mkdir(save_path);
end
% Loop through each figure and save it
for i = 1:length(figHandles)
    % Select the current figure
    figureHandle = figHandles(i);
    % timestamp = datestr(datetime('now'), 'yyyy-mm-dd_HH-MM-SS-FFF');
    % fileName = strcat(save_path,timestamp);
      % Get the title of the figure
    ax = get(figureHandle, 'CurrentAxes'); % Get current axes
    if ~isempty(ax) && ~isempty(get(ax, 'Title')) % Check if title exists
          titleText = get(figureHandle, 'Name'); %Get figure name
        if iscell(titleText)
            titleText = strjoin(titleText, '_'); % Convert cell array to string
        end
    else
        titleText = sprintf('Figure%d', i); % Default name if no title
    end

    % Clean the title to be a valid filename (remove special characters)
    titleText = regexprep(titleText, '[^\w]', '_'); % Replace non-alphanumeric with '_'

    % Construct full file name
    fileName = fullfile(save_path, titleText);
    % Save the figure as a .png file
    % saveas(figureHandle, strcat(fileName,'_',sprintf('figure%d.png', i)));
    print(figureHandle, strcat(fileName, '.png'), '-dpng', '-r300');
    
    % Optionally, save it as a MATLAB .fig file for later editing
    savefig(figureHandle, strcat(fileName,'_',sprintf('figure%d.fig', i)));
    
   
end
 disp('Saved inside:')
 disp(save_path);
end

