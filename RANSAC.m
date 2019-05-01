clear all;close all;
%% Load images and convert to double/grayscale
I1 = im2double(imread('uttower_left.jpg')); 
I2 = im2double(imread('uttower_right.jpg'));
[ylimI1, xlimI1, ~] = size(I1);
[ylimI2, xlimI2, ~] = size(I2);
I1 = rgb2gray(I1);
I2 = rgb2gray(I2);

%% Use Harris algorithm to find feature points and SIFT to formualte
%descriptors
[cim, r1, c1] = harris(I1, 3, 0.03, 3,1); 
[cim2, r2, c2] = harris(I2, 3, 0.03, 3,1);

D = [ c1 r1 20*ones(length(r1),1)];
D2 = [ c2 r2 20*ones(length(r2),1)];

s = find_sift(I1, D, 1.5);
s2 = find_sift(I2, D2, 1.5);
%% Find matching points between images by using a ratio of nearest distance values 
k =1;
for i=1:length(s)
    vals = 0; indx = 0;
    for j=1:length(s2)
        dist(i,j) = dist2(s(i,:), s2(j,:));
    end
    [vals,indx] = sort(dist(i,:));
    ratio = vals(1)/vals(2);
    if ratio < 0.6
        match1(k) = i;
        match2(k) = indx(1);
        mvals(k) = vals(1);
        k=k+1;
    end
end
%% Generate vecotrs of matching points
numMatches = length(match1);
mr1 = r1(match1);
mc1 = c1(match1);
mr2 = r2(match2);
mc2 = c2(match2);

I1pts = [mc1, mr1, ones(numMatches,1)];
I2pts = [mc2, mr2, ones(numMatches,1)];

%% RANSAC finding projective matrix 
iter = 250;
rand = 4;
inlierthresh = 2.6;
inlierind = zeros(numMatches,numMatches);
for i = 1:iter
    randind = randsample(numMatches, rand);
    I1rand = I1pts(randind,:);
    I2rand = I2pts(randind,:);
    
    C = [];
    p = [] ;
    for k=1:rand
        p1x=I1rand(k,1);
        p1y=I1rand(k,2);
        p2x=I2rand(k,1);
        p2y=I2rand(k,2);
        
        Cn = [p1x,p1y,1, 0,0,0,-p1x*p2x, -p1y*p2x; %%Generate C Matrix for all points
            0,0,0,p1x,p1y,1,-p1x*p2y, -p1y*p2y];
        C = [C;Cn];
         
        pi = [p2x;p2y]; %%Generate P matrix all points
        p = [p;pi];
    end
    
    m = pinv(C) * p;
    m(9) = 1;
    M = reshape(m,3,3); %%Shape P into the projective M matrix (3x3) with last entry = 1

    X = I1pts*M;
    X = [X(:,1)./ X(:,3) X(:,2) ./ X(:,3)]; %%Convert from homog
    distx = X(:,1)  - I2pts(:,1)./I2pts(:,3);  %%Residual distances
    disty = X(:,2)  - I2pts(:,2)./I2pts(:,3);
    
    resid = (distx.^2 + disty.^2);  
    
    currind = find(resid<inlierthresh)'; %%Find the residuals < thresh
    inlierind(i,1:length(currind)) = currind; %%Find the index of corresponding inliers
    numinliers(i) = length(find(inlierind(i,:)>0)); 
    resids(i,:) = resid;
end
    
    best = find(numinliers == max(numinliers));%find fit with maximum inliers
    best = best(1);
    
    %% recalculate the line of fit with inliers 
    
    bestinlierind = find(inlierind(best,:)>0);
    bestinlier = inlierind(best,bestinlierind);

    I1in = I1pts(bestinlier,:); %Inlier points
    I2in = I2pts(bestinlier,:);
    numMatchesin = length(I1in);
 %% Plot matched inliers on both images 
matchy = [I1in(:,2), I2in(:,2)];
matchx = [I1in(:,1), I2in(:,1) + xlimI1];
figure; imshow([I1 I2]); hold on; title('Inlier Match Between Images');
hold on; 
plot(I1in(:,1), I1in(:,2),'ys');           
plot(I2in(:,1) + xlimI1, I2in(:,2), 'ys'); 

for i = 1:numMatchesin %%For each match draw a line between    
    plot(matchx(i,:), matchy(i,:));
end
 
 
%% Find projective matrix for all Inliers (Same process of above with RANSAC 
 
 C2 = [];
 p2=[];
 
     for k=1:numMatchesin
        p1x=I1in(k,1);
        p1y=I1in(k,2);
        p2x=I2in(k,1);
        p2y=I2in(k,2);
        
        C2n = [p1x,p1y,1, 0,0,0,-p1x*p2x, -p1y*p2x;
            0,0,0,p1x,p1y,1,-p1x*p2y, -p1y*p2y];
        C2= [C2;C2n];
        
        p2i = [p2x;p2y];
        p2 = [p2;p2i];
  
     end
    m2 = pinv(C2) * p2;
    m2(9) = 1;
    M2 = reshape(m2,3,3);
    
    %%Transform image based on the projective
    homographyTransform = maketform('projective', M2);
    [I1T xtlim ytlim] = imtransform(I1, homographyTransform);
    %% Stitch the images together
    [ylim xlim ~] = size(I2);
   
    [x,y] = meshgrid(xtlim(1):xlim,ytlim(1):ylim); %create grid of all possible points between images
    M2inv = inv(M2);
    
    %Transform all possible points given projection matrix invM *X = x
    xt = (M2inv(1,1)*x + M2inv(2,1)*y + M2inv(3,1)) ./ (M2inv(1,3)*x + M2inv(2,3)*y + M2inv(3,3)) ;  
    yt = (M2inv(1,2)*x + M2inv(2,2)*y + M2inv(3,2)) ./ (M2inv(1,3)*x + M2inv(2,3)*y + M2inv(3,3)) ; 
      
   %Interpolate the image onto the corresponding transformed mesh
   %coordinates
   TLeft = interp2(I1, xt, yt, 'cubic') ;
   Right = interp2(I2, x, y, 'cubic') ;
   
   %Remove useless information
   TLeft(isnan(TLeft)) = 0 ;
   Right(isnan(Right)) = 0 ;
   
   %Average over thbe images
   composite = (TLeft+Right)/2; 
   figure();
   imshow(composite)
   
   %Display usefull information
   display("Number of Inliers = ")
   display(numinliers(best))
   display("Number of Outliers = ")
   display(numMatches - numinliers(best))
   display(mean(resids(best,:)))