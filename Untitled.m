clear
b = [ones(1,8), 3,6,9,12, 4,7,10,13];
b=b+5;
f_gc = ones(16*6,1);
f_gc = reshape(f_gc, 6, length(b));
f_ext = cell(1,21);
for i = unique(b)
   f_ext{i} = sum(f_gc(:,b==i),2);
end