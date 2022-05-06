%% user inputs
% startpoint
prompt='please enter the startpoint in the form [x y theta]: ';
start_node=input(prompt);
% endpoint
prompt='please enter the endpoint in the form[x y theta]: ';
endnode=input(prompt);
% rpm vector
rpm_vec=[50,60];
%% checking for user inputs
while(map(floor(start_node(1)),floor(start_node(2)))==2)
    prompt='the node you entered is in obstacle please enter correct startnode: ';
    start_node=input(prompt);
end
global map
while(map(floor(endnode(1)),floor(endnode(2)))==2)
    prompt='the node you entered is in obstacle please enter correct endnode: ';
    endnode=input(prompt);
end
while(out_bounds(start_node)==1)
    prompt='the node you entered is out of bounds please enter correct startnode: ';
    start_node=input(prompt);
end
while(out_bounds(endnode)==1)
    prompt='the node you entered is out of bounds please enter correct endnode: ';
    endnode=input(prompt);
end
%% parameters for consideration in cm
global r
r=3.5;
global L
L=23;
global clearance
clearance=210;
t=1;


%% Algorithm (A*)
node=1;
res=10;
visited_arr=zeros(floor(11100/res),floor(10100/res));
global visited_a
visited_a=[];
cost_arr =inf(floor(11100/res),floor(10100/res));
cost_arr(1,1)=0;
cost_goarr=zeros(floor(11100/res),floor(10100/res));
cost_comearr=zeros(floor(11100/res),floor(10100/res));
parent_arr=cell(floor(11100/res),floor(10100/res));
rpm_arr=cell(floor(11100/res),floor(10100/res));
prune_arr=zeros(111,101);
q=[];
q=start_node;
z=start_node;
while(endcircle(endnode,z)==0)
    %marking startnode as visited
    visited_arr(start_node(1),start_node(2))=1;
    [z,q]=pop(q,cost_arr);
    [status,newnode]= movestraight(z,rpm_vec,t,map);
    if(status==1&&prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))==0)
        prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))=1;
        rpm_arr(newnode(1),newnode(2))={[rpm_vec(1) rpm_vec(1)]};
        value=visited(visited_arr,newnode);
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            % update the parent
            parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};
            [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
            cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+((2*rpm_vec(1)*pi*r*t)/60);
            cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
        else
            if(cost_comearr(newnode(1),newnode(2))>cost_comearr(z(1),z(2))+((2*rpm_vec(1)*pi*r*t)/60))
               [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
               cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+((2*rpm_vec(1)*pi*r*t)/60);
               cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
               parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};   
            end
               
        end
    end
    
    [status,newnode]= movestraight_l(z,rpm_vec,t,map);
    if (status==1&&prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))==0)
        prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))=1;
        value=visited(visited_arr,newnode);
        rpm_arr(newnode(1),newnode(2))={[rpm_vec(2) rpm_vec(2)]};
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};
            [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
            cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+((2*rpm_vec(2)*pi*r*t)/60);
            cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
            
        else
            if(cost_comearr(newnode(1),newnode(2))>cost_comearr(z(1),z(2))+((2*rpm_vec(2)*pi*r*t)/60))
                [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
                cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+((2*rpm_vec(2)*pi*r*t)/60);
                cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
                parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};
              
            end
       end
         
    end
    
    [status,newnode]= movemoreleft(z,rpm_vec,t,map);
    if (status==1&&prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))==0)
        prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))=1;
        value=visited(visited_arr,newnode);
        rpm_arr(newnode(1),newnode(2))={[0 rpm_vec(1)]};
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};
            [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
            cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+((rpm_vec(1)*pi*r*t)/60);
            cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
            
        else
            if(cost_comearr(newnode(1),newnode(2))>cost_comearr(z(1),z(2))+((rpm_vec(1)*pi*r*t)/60))
                [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
                cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+((rpm_vec(1)*pi*r*t)/60);
                cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
                parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};
                
            end
        end
            
    end
    
    [status,newnode]= movemoreleft_l(z,rpm_vec,t,map);
    if (status==1&&prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))==0)
        prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))=1;
        value=visited(visited_arr,newnode);
        rpm_arr(newnode(1),newnode(2))= {[0 rpm_vec(2)]};
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};
            [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
            cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+((rpm_vec(2)*pi*r*t)/60);
            cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;      
            
        else
            if(cost_comearr(newnode(1),newnode(2))>cost_comearr(z(1),z(2))+((rpm_vec(2)*pi*r*t)/60))
                [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
                cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+((rpm_vec(2)*pi*r*t)/60);
                cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
                parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};
               
            end
        end
    end
    
    [status,newnode]= movemoreright(z,rpm_vec,t,map);
    if (status==1&&prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))==0)
        prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))=1;
        value=visited(visited_arr,newnode);
        rpm_arr(newnode(1),newnode(2))={[rpm_vec(1) 0]};
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};
            [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
            cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+((rpm_vec(1)*pi*r*t)/60);
            cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
             
             
         else
            if(cost_comearr(newnode(1),newnode(2))>cost_comearr(z(1),z(2))+((rpm_vec(1)*pi*r*t)/60))
                [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
                cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+((rpm_vec(1)*pi*r*t)/60);
                cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
                parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};
            end
         end
    end
    
    [status,newnode]= movemoreright_l(z,rpm_vec,t,map);
    if (status==1&&prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))==0)
        prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))=1;
        value=visited(visited_arr,newnode);
        rpm_arr(newnode(1),newnode(2))={[rpm_vec(2) 0]};
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};
            [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
            cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+((rpm_vec(2)*pi*r*t)/60);
            cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
            
            
        else
            if(cost_comearr(newnode(1),newnode(2))>cost_comearr(z(1),z(2))+((rpm_vec(2)*pi*r*t)/60))
                [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
                cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+((rpm_vec(2)*pi*r*t)/60);
                cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
                parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};
               
            end
        end
    end
    [status,newnode]= movebit_left(z,rpm_vec,t,map);
    if (status==1&&prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))==0)
        prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))=1;
        value=visited(visited_arr,newnode);
        rpm_arr(newnode(1),newnode(2))={[rpm_vec(1) rpm_vec(2)]};
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};
            [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
            cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+(((rpm_vec(1)+rpm_vec(2))*pi*r*t)/60);
            cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
           
            
        else
            if( cost_comearr(newnode(1),newnode(2))>cost_comearr(z(1),z(2))+(((rpm_vec(1)+rpm_vec(2))*pi*r*t)/60))
                [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
                cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+(((rpm_vec(1)+rpm_vec(2))*pi*r*t)/60);
                cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
                parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};
            end
        end
    end
    [status,newnode]= movebit_right(z,rpm_vec,t,map);
    if (status==1&&prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))==0)
        prune_arr(floor(newnode(1)/10),floor(newnode(2)/10))=1;
        value=visited(visited_arr,newnode);
        rpm_arr(newnode(1),newnode(2))={[rpm_vec(2) rpm_vec(1)]};
        if (value==0)
            visited_arr(newnode(1),newnode(2))=1;
            visited_a=cat(1,visited_a,newnode);
            q=insert(q,newnode);
            parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};
            [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
            cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+(((rpm_vec(1)+rpm_vec(2))*pi*r*t)/60);
            cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
            
        else
            if(cost_comearr(newnode(1),newnode(2))>cost_comearr(z(1),z(2))+(((rpm_vec(1)+rpm_vec(2))*pi*r*t)/60))
                [c,cost_goarr]=cost_go(cost_goarr,newnode,endnode);
                cost_comearr(newnode(1),newnode(2))=cost_comearr(z(1),z(2))+(((rpm_vec(1)+rpm_vec(2))*pi*r*t)/60);
                cost_arr(newnode(1),newnode(2))=cost_comearr(newnode(1),newnode(2))+c;
                parent_arr(newnode(1),newnode(2))={[z(1) z(2) z(3)]};               
            end
        end
       
    end
end
%% backtracking
a=z;
j=z;
while(isequal(j,[start_node(1),start_node(2),start_node(3)])~=1)
    temp=parent_arr(j(1),j(2));
    j=cell2mat(temp);
    a=cat(1,a,j);
end
global path
a=fliplr(a');
path=a';
path=path;
[row,col]=find(visited_arr==1);
b=cat(2,row,col);
% finding rpms
path_rpm=zeros(1,2);
for i= 2:size(path,1)
path_rpm = cat(1,path_rpm,cell2mat(rpm_arr(path(i,1),path(i,2))));
end
path_rpm(1,:)=[];


%% draw a circle around goal node
function [f]= endcircle(endnode,nod)
end_circle=(nod(1)-endnode(1))^2 + (nod(2)-endnode(2))^2 -(10)^2;
if(end_circle<=0)
    f=1;
else
    f=0;
end
end
%% cost to go computing
function[c,cost_goarr]=cost_go(cost_goarr,newnode,endnode)
%euclidean distance
cost_goarr(newnode(1),newnode(2))=sqrt((newnode(1)-endnode(1))^2 + (newnode(2)-endnode(2))^2);
c=cost_goarr(newnode(1),newnode(2));
end
%% insert
 function [q]= insert(q,newnode)
 q=cat(1,q,newnode);
 end
 
 %% pop
 function [inst,s]= pop(s,cost_arr)
 min=1;
 minx=s(1,1);
 miny=s(1,2);
 for i=1:size(s,1)
     x=s(i,1);
     y=s(i,2);
     if(cost_arr(x,y)<cost_arr(minx,miny))
         min=i;
         minx=x;
         miny=y;
         inst=s(min,:);
        %delete the element from queue
     end
         
 end
 inst=s(min,:);
 s(min,:)=[];
 end
%% check wether visited or not
function [value]= visited(visited_arr,newnode)
if(visited_arr(newnode(1),newnode(2))==1)
    value=1;
else
    value=0;
end
end


%% outof bounds check
function[flag]= out_bounds(newnode)
global clearance
if(newnode(1)<=floor((0+(clearance/10)))||newnode(2)<=floor((0+(clearance/10)))||newnode(1)>=floor((1110-(clearance/10)))||newnode(2)>=floor((1010-(clearance/10))))
    flag=1; % out of bounds
else
    flag=0;
end
end
%% action set
% for the case with [rpm1 rpm1]move straight
function[status,newnode]= movestraight(currentnode,rpm_vec,t,map)
global r
th=currentnode(3);
dx=0;
dy=0;
    for k=1:100
        dx=dx+(2*pi*(r/60)*(rpm_vec(1))*cosd(th)*(t/100));
        dy=dy+(2*pi*(r/60)*(rpm_vec(1))*sind(th)*(t/100));
        if out_bounds([currentnode(1)+dx,currentnode(2)+dy])==0
            if (map(round(currentnode(1)+dx),round(currentnode(2)+dy))==2)
                 status=0;
            end
        end
    end
newnode(3)=currentnode(3);
newnode(1)=round(currentnode(1)+dx);
newnode(2)=round(currentnode(2)+dy);
   if out_bounds(newnode)==0
       if (map(newnode(1),newnode(2))==0)
       status=1;
       else
       newnode=[];
       status=0;
       end
   else
       newnode=[];
       status=0;
   end
end
% for [rpm2 rpm2]
function[status,newnode]= movestraight_l(currentnode,rpm_vec,t,map)
global r
th=currentnode(3);
dx=0;
dy=0;
    for k=1:100
        dx=dx+(2*pi*(r/60)*(rpm_vec(1))*cosd(th)*(t/100));
        dy=dy+(2*pi*(r/60)*(rpm_vec(1))*sind(th)*(t/100));
        if out_bounds([currentnode(1)+dx,currentnode(2)+dy])==0
            if (map(round(currentnode(1)+dx),round(currentnode(2)+dy))==2)
                 status=0;
            end
        end
    end
newnode(3)=currentnode(3);
newnode(1)=round(currentnode(1)+dx);
newnode(2)=round(currentnode(2)+dy);
    if out_bounds(newnode)==0
       if (map(newnode(1),newnode(2))==0)
       status=1;
       else
       newnode=[];
       status=0;
       end
   else
       newnode=[];
       status=0;
    end
end
% for case with [0 rpm1]
function[status,newnode]= movemoreleft(currentnode,rpm_vec,t,map)
global r
global L
th=currentnode(3);
dx=0;
dy=0;
    for k=1:100
        dx=dx+(2*pi*(r/120)*(rpm_vec(1))*cosd(th)*(t/100));
        dy=dy+(2*pi*(r/120)*(rpm_vec(1))*sind(th)*(t/100));
        dtheta=6*(r/L)*(rpm_vec(1))*(t/100);
        th=th+dtheta;
        if out_bounds([currentnode(1)+dx,currentnode(2)+dy])==0
            if (map(round(currentnode(1)+dx),round(currentnode(2)+dy))==2)
                 status=0;
            end
        end
    end
newnode(3)=th;
newnode(1)=round(currentnode(1)+dx);
newnode(2)=round(currentnode(2)+dy);
    if out_bounds(newnode)==0
       if (map(newnode(1),newnode(2))==0)
       status=1;
       else
       newnode=[];
       status=0;
       end
   else
       newnode=[];
       status=0;
   end
end
% for [0 rpm2]
function[status,newnode]= movemoreleft_l(currentnode,rpm_vec,t,map)
global r
global L
th=currentnode(3);
dx=0;
dy=0;
    for k=1:100
        dx=dx+(2*pi*(r/120)*(rpm_vec(2))*cosd(th)*(t/100));
        dy=dy+(2*pi*(r/120)*(rpm_vec(2))*sind(th)*(t/100));
        dtheta=6*(r/L)*(rpm_vec(2))*(t/100);
        th=th+dtheta;
        if out_bounds([currentnode(1)+dx,currentnode(2)+dy])==0
            if (map(round(currentnode(1)+dx),round(currentnode(2)+dy))==2)
                 status=0;
            end
        end
    end
newnode(3)=th;
newnode(1)=round(currentnode(1)+dx);
newnode(2)=round(currentnode(2)+dy);
    if out_bounds(newnode)==0
       if (map(newnode(1),newnode(2))==0)
       status=1;
       else
       newnode=[];
       status=0;
       end
   else
       newnode=[];
       status=0;
   end
  
end
% for the case with [rpm1 0]
function[status,newnode]= movemoreright(currentnode,rpm_vec,t,map)
global r
global L
th=currentnode(3);
dx=0;
dy=0;
    for k=1:100
        dx=dx+(2*pi*(r/120)*(rpm_vec(1))*cosd(th)*(t/100));
        dy=dy+(2*pi*(r/120)*(rpm_vec(1))*sind(th)*(t/100));
        dtheta=-6*(r/L)*(rpm_vec(1))*(t/100);
        th=th+dtheta;
        if out_bounds([currentnode(1)+dx,currentnode(2)+dy])==0
            if (map(round(currentnode(1)+dx),round(currentnode(2)+dy))==2)
                 status=0;
            end
        end
    end
newnode(3)=th;
newnode(1)=round(currentnode(1)+dx);
newnode(2)=round(currentnode(2)+dy);
    if out_bounds(newnode)==0
       if (map(newnode(1),newnode(2))==0)
       status=1;
       else
       newnode=[];
       status=0;
       end
   else
       newnode=[];
       status=0;
   end
end
% for [rpm2 0]
function[status,newnode]= movemoreright_l(currentnode,rpm_vec,t,map)
global r
global L
th=currentnode(3);
dx=0;
dy=0;
    for k=1:100
        dx=dx+(2*pi*(r/120)*(rpm_vec(2))*cosd(th)*(t/100));
        dy=dy+(2*pi*(r/120)*(rpm_vec(2))*sind(th)*(t/100));
        dtheta=-6*(r/L)*(rpm_vec(2))*(t/100);
        th=th+dtheta;
        if out_bounds([currentnode(1)+dx,currentnode(2)+dy])==0
            if (map(round(currentnode(1)+dx),round(currentnode(2)+dy))==2)
                 status=0;
            end
        end
    end
newnode(3)=th;
newnode(1)=round(currentnode(1)+dx);
newnode(2)=round(currentnode(2)+dy);
    if out_bounds(newnode)==0
       if (map(newnode(1),newnode(2))==0)
       status=1;
       else
       newnode=[];
       status=0;
       end
   else
       newnode=[];
       status=0;
   end
end

% for [rpm1 rpm2]
function[status,newnode]= movebit_left(currentnode,rpm_vec,t,map)
global r
global L
th=currentnode(3);
dx=0;
dy=0;
    for k=1:100
        dx=dx+(2*pi*(r/120)*(rpm_vec(2)+rpm_vec(1))*cosd(th)*(t/100));
        dy=dy+(2*pi*(r/120)*(rpm_vec(2)+rpm_vec(1))*sind(th)*(t/100));
        dtheta=6*(r/L)*(rpm_vec(2)-rpm_vec(1))*(t/100);
        th=th+dtheta;
        if out_bounds([currentnode(1)+dx,currentnode(2)+dy])==0
            if (map(round(currentnode(1)+dx),round(currentnode(2)+dy))==2)
                 status=0;
            end
        end
    end
newnode(3)=th;
newnode(1)=round(currentnode(1)+dx);
newnode(2)=round(currentnode(2)+dy);
    if out_bounds(newnode)==0
       if (map(newnode(1),newnode(2))==0)
       status=1;
       else
       newnode=[];
       status=0;
       end
   else
       newnode=[];
       status=0;
   end
end
%for [rpm2 rpm1]
function[status,newnode]= movebit_right(currentnode,rpm_vec,t,map)
global r
global L
th=currentnode(3);
dx=0;
dy=0;
    for k=1:100
        dx=dx+(2*pi*(r/120)*(rpm_vec(2)+rpm_vec(1))*cosd(th)*(t/100));
        dy=dy+(2*pi*(r/120)*(rpm_vec(2)+rpm_vec(1))*sind(th)*(t/100));
        dtheta=6*(r/L)*(rpm_vec(1)-rpm_vec(2))*(t/100);
        th=th+dtheta;
        if out_bounds([currentnode(1)+dx,currentnode(2)+dy])==0
            if (map(round(currentnode(1)+dx),round(currentnode(2)+dy))==2)
                 status=0;
            end
        end
    end
newnode(3)=th;
newnode(1)=round(currentnode(1)+dx);
newnode(2)=round(currentnode(2)+dy);
   if out_bounds(newnode)==0
       if (map(newnode(1),newnode(2))==0)
       status=1;
       else
       newnode=[];
       status=0;
       end
   else
       newnode=[];
       status=0;
   end
end