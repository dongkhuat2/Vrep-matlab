    % obstacle space
    clc;
    clear all;
    res=10;
    global map
    map=zeros(floor(10100/res),floor(11100/res));
    % axis([0,floor(11100/res),0,floor(10100/res)])
    for k=1:floor(10100/res)
        for l=1:floor(11100/res)
            if(obspace(k,l,res)==1)
                map(k,l)=2;
            else
                map(k,l)=0;
            end
        end
    end
    map=rot90(map,-1);

    Visualize(map)
    function Visualize(map)
    res = 1;
    cmap = [1 1 1; ...
            0 0 0; ...
            1 0 0; ...
            0 0 1; ...
            0 1 0; ...
            1 1 0; ...
        0.5 0.5 0.5];
    colormap(cmap);
    map = map';
    image([0 1110],[0 1010],map); 
    hold on;
    %grid on
    set(gca,'ydir','normal'); 
    set(gca,'XTick',0:50:1110,'YTick',0:50:1010);
    end
    function Cobs= obspace(x,y,res)
    % set clearance here
    global clearance
    clearance=210;
    % circle pillars(taking robot radius plus clearance as 200mm)
    c1=(x-floor(450/res))^2 + (y-floor(3900/res))^2 -(605/res)^2;
    c2=(x-floor(2740/res))^2 + (y-floor(4380/res))^2 -(605/res)^2;
    c3=(x-floor(7360/res))^2 + (y-floor(4380/res))^2 -(605/res)^2;
    c4=(x-floor(9650/res))^2 + (y-floor(3900/res))^2 -(605/res)^2;
    %tables
    % t1
    h1=-y+((8320-clearance)/res);
    h2=x-((1830+clearance)/res);
    h3=y-((9180+clearance)/res);
    h4=-x;
    if(h1<=0 && h2<=0 && h3<=0 && h4<=0)
        t1=1;
    else
        t1=0;
    end
    % t2
    h5=-y+((9830-clearance)/res);
    h6=x-((910+clearance)/res);
    h7=y-((10260+clearance)/res);
    h8=-x;
    if(h5<=0 && h6<=0 && h7<=0 && h8<=0)
        t2=1;
    else
        t2=0;
    end
    % t3
    h9=-y+((7440-clearance)/res);
    h10=x-((3890+clearance)/res);
    h11=y-(11100/res);
    h12=-x+((3130-clearance)/res);
    if(h9<=0 && h10<=0 && h11<=0 && h12<=0)
        t3=1;
    else
        t3=0;
    end
    % t4
    h13= -y+((10520-clearance)/res);
    h14= x-(5610/res);
    h15= y-(11100/res);
    h16= -x+((4445-clearance)/res);
    if(h13<=0 && h14<=0 && h15<=0 && h16<=0)
        t4=1;
    else
        t4=0;
    end

    % t5
    h17= -y+((10190-clearance)/res);
    h18= x-((6475+clearance)/res);
    h19=y-(11100/res);
    h20=-x+(5610/res);
    if(h17<=0 && h18<=0 && h19<=0 && h20<=0)
        t5=1;
    else
        t5=0;
    end
    % t6
    h21= -y+((10520-clearance)/res);
    h22= x-((8317+clearance)/res);
    h23=y-(11100/res);
    h24=-x+((7147-clearance)/res);
    if(h21<=0 && h22<=0 && h23<=0 && h24<=0)
        t6=1;
    else
        t6=0;
    end

    % t7
    h25=-y+((9270-clearance)/res);
    h26=x-((9750+clearance)/res);
    h27=y-(11100/res);
    h28=-x+((8990-clearance)/res);
    if(h25<=0 && h26<=0 && h27<=0 && h28<=0)
        t7=1;
    else
        t7=0;
    end
    % t8
    h29=-y+((7790-clearance)/res);
    h30=x-(9750/res);
    h31=y-((8960+clearance)/res);
    h32=-x+((9170-clearance)/res);
    if(h29<=0 && h30<=0 && h31<=0 && h32<=0)
        t8=1;
    else
        t8=0;
    end
    % t9
    h33=-y+((4740-clearance)/res);
    h34=x-(9750+clearance/res);
    h35=y-((7480+clearance)/res);
    h36=-x+((8230-clearance)/res);
    if(h33<=0 && h34<=0 && h35<=0 && h36<=0)
        t9=1;
    else
        t9=0;
    end
    % t10
    h37=-y+((6850-clearance)/res);
    h38=x-((10100)/res);
    h39=y-(11100/res);
    h40=-x+(9750/res);
    if(h37<=0 && h38<=0 && h39<=0 && h40<=0)
        t10=1;
    else
        t10=0;
    end
    % t11
    h41=-y+((7845-clearance)/res);
    h42=x-((7430+clearance)/res);
    h43=y-((9365+clearance)/res);
    h44=-x+((6260-clearance)/res);
    if(h41<=0 && h42<=0 && h43<=0 && h44<=0)
        t11=1;
    else
        t11=0;
    end
    % t12
    h45=-y+((5290-clearance)/res);
    h46=x-((7450+clearance)/res);
    h47=y-((7120+clearance)/res);
    h48=-x+((6690-clearance)/res);
    if(h45<=0 && h46<=0 && h47<=0 && h48<=0)
        t12=1;
    else
        t12=0;
    end
    % t13
    h49=-y+((4380-clearance)/res);
    h50=x-((6950+clearance)/res);
    h51=y-((5290+clearance)/res);
    h52=-x+((5120-clearance)/res);
    if(h49<=0 && h50<=0 && h51<=0 && h52<=0)
        t13=1;
    else
        t13=0;
    end
    % elliptical table
    e1=(x-floor(1799/res))^2 + (y-floor(1500/res))^2 -(1000/res)^2;
    e2=x-((2599+clearance)/res);
    e3=(x-floor(1799/res))^2 + (y-floor(3097/res))^2 -(1000/res)^2;
    e4=-x+((1000-clearance)/res);
    e5=-y+((1499-clearance)/res);
    e6=y-((3097+clearance)/res);
    if(e1<=0||e3<=0||(e5<=0&&e6<=0&&e2<=0&&e4<=0))
        et=1;
    else
        et=0;
    end
    % boundaries of map
    h53=y-(clearance/res);
    h54=-y;
    h55=-x+((10100-clearance)/res);
    h56=x-(10100/res);
    h57=y-(11100/res);
    h58=-y+((11100-clearance)/res);
    h59=x-(clearance/res);
    if((h53<=0 && h54<=0)||(h55<=0&&h56<=0)||(h57<=0&&h58<=0)||h59<=0)
        t14=1;
    else
        t14=0;
    end

    if(c1<=0||c2<=0||c3<=0||c4<=0||t1==1||t2==1||t3==1||t4==1||t5==1||t6==1||t7==1||t8==1||t9==1||t10==1||t11==1||t12==1||t13==1||et==1||t14==1)
        Cobs=1;
    else
        Cobs=0;
    end
    end
