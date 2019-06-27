% Credits : Tim Bailey and Juan Nieto 2004. 
function data= store_data(data,xtrue)
% add current data to offline storage
% CHUNK= 5000;
% if data.i == size(data.path,2) % grow array in chunks to amortise reallocation
%     data.path= [data.path zeros(3,CHUNK)];
%     data.true= [data.true zeros(3,CHUNK)];
% end
i= data.i + 1;
data.i= i;
data.true(:,i)= xtrue;
end
