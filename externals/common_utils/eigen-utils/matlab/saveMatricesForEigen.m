function saveMatricesForEigen(fname, mats, type)
if (nargin<3)
    type = 'double';
end
fid = fopen(fname,'w');

if (strcmp(type,'float'))
    savetype = 'single';
elseif (strcmp(type,'double'))
    savetype = 'double';
elseif (strcmp(type,'int32_t'))
    savetype = 'int32';
else
    error(['invalid dataType: ' type]);
end
typelen =length(type);

if (~iscell(mats))
    mats = {mats};
end

for i=1:length(mats)
    [rows cols] = size(mats{i});
    written = fwrite(fid,rows,'int32');
    written = fwrite(fid,cols,'int32');
    written = fwrite(fid,typelen,'int32');
    written = fwrite(fid,type,'char');
    assert(written == typelen)
    written = fwrite(fid,mats{i},savetype);
end
fclose(fid);

end