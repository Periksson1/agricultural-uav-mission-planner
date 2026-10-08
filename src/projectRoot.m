function root = projectRoot()
%PROJECTROOT Resolve project files independently of the current folder.
root = fileparts(fileparts(mfilename('fullpath')));
end
