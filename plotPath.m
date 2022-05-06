
hold on
for i=1:size(path,1)
plot(path(i,1), path(i,2),'b*');pause(0.0001);
end

hold on
for i=1:size(visited_a,1)
rectangle('Position',[visited_a(i,1) visited_a(i,2) 1 1], 'FaceColor','yellow','EdgeColor','r');pause(0.0001);
end
hold on
% plot(1000,200,'r*')
% plot(1000,850,'b*')
