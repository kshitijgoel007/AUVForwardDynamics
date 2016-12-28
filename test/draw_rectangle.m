function v=draw_rectangle()
n=1;
for i=-2:.1:2
    v(n,1)=i;
    v(n,2)=1;
    v(n,3)=0;
    n=n+1;
end

for i=-2:.1:2
    v(n,1)=i;
    v(n,2)=-1;
    v(n,3)=0;
    n=n+1;
end

for j=-1:.1:1
    v(n,1)=-2;
    v(n,2)=j;
    v(n,3)=0;
    n=n+1;
end

for j=-1:.1:1
    v(n,1)=2;
    v(n,2)=j;
    v(n,3)=0;
    n=n+1;
end

end