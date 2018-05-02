function [match] = match_lines(target, template)

% generate points from template
step = 5;

template_points = cell(size(template, 1), 1);
template_labels = cell(size(template, 1), 1);
for i = 1:size(template, 1)
    len = round(sqrt(sum((template(i, 1:2)-template(i, 3:4)).^2, 2)));
    template_points{i} = cell2mat(arrayfun(@(x) template(i,1:2)*x + template(i,3:4)*(1-x),...
        (0:step/len:1)', 'UniformOutput', false));
    template_labels{i} = i*ones(size(template_points{i}, 1), 1);
end
template_points = cell2mat(template_points);
template_labels = cell2mat(template_labels);

target_points = cell(size(target, 1), 1);
target_labels = zeros(size(target, 1)+1, 1);
for i = 1:size(target, 1)
    len = round(sqrt(sum((target(i, 1:2)-target(i, 3:4)).^2, 2)));
    target_points{i} = cell2mat(arrayfun(@(x) target(i,1:2)*x + target(i,3:4)*(1-x),...
        (0:step/len:1)', 'UniformOutput', false));
    target_labels(i+1) = target_labels(i) + size(target_points{i}, 1);
end
target_points = cell2mat(target_points);

idxs = rangesearch(template_points, target_points, 3);
match = zeros(size(target, 1), 1);
for i = 1:size(target, 1)
    idx = idxs(target_labels(i)+1:target_labels(i+1));
    n_samples = numel(idx);
    idx = idx(~cellfun('isempty',idx))';
    labels = template_labels([idx{:}]');
    if numel(labels) == 0
        continue;
    end
    [M,F] = mode(labels);
    if F > n_samples*0.6
        match(i) = M;
    end
end

end



