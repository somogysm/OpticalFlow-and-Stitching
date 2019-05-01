function [u,v] = opticalFlow(I1,I2,windowSize)

win = windowSize(1); win2 = windowSize(2); %Generate window variable 
I1 = im2double(imread(I1)); %Load image files as type double
I2 = im2double(imread(I2));

%% Show images and hold for velocity vectors
figure();
subplot(1,2,1);imshow(I1);
subplot(1,2,2);imshow(I2);hold on;
%%Check that the iamges are not in grayscale, if not convert. 
if size(I1,3)==3    %Convert to grayscale if not already.
    I1 = rgb2gray(I1);
end
if size(I2,3)==3
    I2 = rgb2gray(I2);
end
%% Scale the Intensities  of each image between 0 and 1
I1 = (I1 - min(min(I1)))/(max(max(I1))- min(min(I1)));
I2 = (I2 - min(min(I2)))/(max(max(I2))- min(min(I2)));

%% Find the partial derivative in the x and y directions of Image 1, as well as with respect to time 
Ixm = conv2(I1,[-1 1; -1 1], 'valid');  % partial on x
Iym = conv2(I1, [-1 -1; 1 1], 'valid');  % partial on y
Itm = conv2(I1, ones(2), 'valid') + conv2(I2, -ones(2), 'valid'); % partial on t

%% Generate A matrix within the given window values.
for i = win+1:size(Ixm,1)-win  
   for j = win2+1:size(Ixm,2)-win2
      Ix = Ixm(i-win:i+win, j-win2:j+win2)';
      Iy = Iym(i-win:i+win, j-win2:j+win2)';
      It = Itm(i-win:i+win, j-win2:j+win2)';
          
      A = [Ix(:) Iy(:)]; 
     
      e = eig(A'*A);
      t = 0.01;
      if min(e(1),e(2)) < t   %exclude eigenvalues below the threshold of t
          A = 0; % Optical flow at e<t not calculated ( = 0)
      end
          
      V = pinv(A'*A)*A'*-It(:);  %Use the psuedo inverse to find the displacement matrix A'A U = A'B (solve U)
      u(i,j)=V(1); 
      v(i,j)=V(2);
      
   end;
end;
%% Map the displacements to original image size minus the window
% Only every Jth vector was displayed for visibility. (arbitrarily chosen)
j = length(u)/20;
us = u(1:j:end, 1:j:end);
vs = v(1:j:end, 1:j:end);
[X,Y] = meshgrid(1:length(u), 1:length(u)); %Mesh of all possible values in the Image (minus window) 
Xs = X(1:j:end, 1:j:end);
Ys = Y(1:j:end, 1:j:end);

quiver(Xs, Ys, us,vs,'r') %plot using quiver on the mesh (overlayed on the held image 2 from above)

end 

