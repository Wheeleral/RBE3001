% HOLY MOSES THAT'S A LOT OF CODE, you may say
% That's why I'm here; to walk you through the mess of a thing image
% processing is.
% Buckle up, we're going for a ride.


function u = visionTest(cam)


% First, takes a picture of the field
img = snapshot(cam); %Takes the image
img = imcrop(img, [476.5 429.5 997 455]); %Crops it to useable area
%img = imcrop(img);
weights = weightMask(img); % Finds where weights are

% This finds certain colors and shapes to sort and locate objects
% It's super ugly, mostly because images don't play nice with arrays in
% Matlab
imgM = maskBlue(img); % Finds all blue objects
imgN = imgM + weights; % Makes an image of just blue objects and weights
imgN = boolean(bwlabel(imgN)); % Flattens to B&W image
imgN = imdilate(imgN, strel('line',10,0)); % Processes image to return only objects that are a) blue, and b) have weights
imgN = imfill(imgN, 'holes');
BWL = bwareafilt(imgN,[20000 30000]);
imgBlueM = regionprops(BWL, 'centroid'); % Returns centroid location of blue weighted objects

imgM= xor(boolean(bwlabel(imgM)),BWL); % Finds blue objects that do not have weights
BWL = bwareafilt(imgM,[1000 10000]);
imgBlue = regionprops(BWL, 'centroid'); % Returns those so they don't get lonely

% Now we do it all over again, but with yellow!
% Same code but with "Yellow" instead of "Blue"
% Riveting, truly.
imgM = maskYellow(img);
imgN = imgM + weights;
imgN = boolean(bwlabel(imgN));
imgN = imdilate(imgN, strel('line',10,0));
imgN = imfill(imgN, 'holes');
BWL = bwareafilt(imgN,[20000 30000]);
imgYellowM = regionprops(BWL, 'centroid');

imgM= xor(boolean(bwlabel(imgM)),BWL);
BWL = bwareafilt(imgM,[1000 10000]);
imgYellow = regionprops(BWL, 'centroid');

% One more time, but with Green
% Seriously Matlab, why don't you let me put images in arrays?
imgM = maskGreen(img);
imgN = imgM + weights;
imgN = boolean(bwlabel(imgN));
imgN = imdilate(imgN, strel('line',10,0));
imgN = imfill(imgN, 'holes');
BWL = bwareafilt(imgN,[20000 30000]);
imgGreenM = regionprops(BWL, 'centroid');

imgM= xor(boolean(bwlabel(imgM)),BWL);
BWL = bwareafilt(imgM,[1000 10000]);
imgGreen = regionprops(BWL, 'centroid');

returnArray=[];

% Overlays centroid on the original image
imshow(img);
hold on

if ~isempty(imgBlue) % Checks to see if blue, unweighted objects exist
    centroid = cat(1,imgBlue.Centroid); % If so, finds centroid location
    plot(centroid(:,1),centroid(:,2), 'yh', 'MarkerSize', 10, 'LineWidth', 4); % Plots location on image
    hold off
    disp(centroid);
    
    returnArray = [ returnArray; [transformCentroid(centroid) 1]]
    
end
% if ~isempty(imgBlueM) % Checks to see if blue, weighted objects exist
%     hold on
%     centroid = cat(1,imgBlueM.Centroid);
%     plot(centroid(:,1),centroid(:,2)-40, 'y+', 'MarkerSize', 10, 'LineWidth', 4);
%     hold off
%     disp(centroid);
%     returnArray = cat(1, returnArray, cat(2, centroid, 'blue'));
% end
if ~isempty(imgYellow) % Yellow, unweighted
    hold on
    centroid = cat(1,imgYellow.Centroid);
    plot(centroid(:,1),centroid(:,2), 'bh', 'MarkerSize', 10, 'LineWidth', 4); %plot the cnetorids on the image
    hold off
    disp(centroid);
    
    returnArray = [ returnArray; [transformCentroid(centroid) 3]];
end

% if ~isempty(imgYellowM) % Yellow, weighted
%     hold on
%     centroid = cat(1,imgYellowM.Centroid);
%     plot(centroid(:,1),centroid(:,2)-40, 'b+', 'MarkerSize', 10, 'LineWidth', 4); %plot the cnetorids on the image
%     hold off
%     disp(centroid);
%     returnArray = cat(1, returnArray, cat(2, centroid, 'yellow'));
% end
if ~isempty(imgGreen) % Green, unweighted
    hold on
    centroid = cat(1,imgGreen.Centroid);
    plot(centroid(:,1),centroid(:,2), 'rh', 'MarkerSize', 10, 'LineWidth', 4); %plot the cnetorids on the image
    hold off
    disp(centroid);
    returnArray = [returnArray;[transformCentroid(centroid) 2 ]];
    
end
% if ~isempty(imgGreenM) % Green, weighted
%     hold on
%     centroid = cat(1,imgGreenM.Centroid);
%     plot(centroid(:,1),centroid(:,2)-40, 'r+', 'MarkerSize', 10, 'LineWidth', 4); %plot the cnetorids on the image
%     hold off
%     disp(centroid);
%     returnArray = cat(1, returnArray, cat(2, centroid, 'green'));
% end
if (length(returnArray)>0)
    u = returnArray;
else
    u = 'null';
end
end


% ABANDON HOPE, ALL YE WHO VENTURE BELOW
% Everything below here was generated by Matlab as masking functions
% That's all it does. There are 4 masks; blue, yellow, green, and weight
% Yes I know weight isn't a color, don't judge me
% Read the code below if you are bored and want to become more bored


function [BW,maskedRGBImage] = maskBlue(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder App. The colorspace and
%  minimum/maximum values for each channel of the colorspace were set in the
%  App and result in a binary mask BW and a composite image maskedRGBImage,
%  which shows the original RGB image values under the mask BW.

% Auto-generated by colorThresholder app on 02-Oct-2017
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.561;
channel1Max = 0.660;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.000;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.282;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;



end

function [BW,maskedRGBImage] = maskYellow(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder App. The colorspace and
%  minimum/maximum values for each channel of the colorspace were set in the
%  App and result in a binary mask BW and a composite image maskedRGBImage,
%  which shows the original RGB image values under the mask BW.

% Auto-generated by colorThresholder app on 02-Oct-2017
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.158;
channel1Max = 0.199;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.543;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.718;
channel3Max = 0.941;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

end

function [BW,maskedRGBImage] = maskGreen(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder App. The colorspace and
%  minimum/maximum values for each channel of the colorspace were set in the
%  App and result in a binary mask BW and a composite image maskedRGBImage,
%  which shows the original RGB image values under the mask BW.

% Auto-generated by colorThresholder app on 02-Oct-2017
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.340;
channel1Max = 0.377;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.537;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.186;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

end

function [BW,maskedRGBImage] = weightMask(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder App. The colorspace and
%  minimum/maximum values for each channel of the colorspace were set in the
%  App and result in a binary mask BW and a composite image maskedRGBImage,
%  which shows the original RGB image values under the mask BW.

% Auto-generated by colorThresholder app on 05-Oct-2017
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.099;
channel1Max = 0.143;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.500;
channel2Max = 0.798;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.324;
channel3Max = 0.697;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

end

function u = transformCentroid(centroid)

[x, y]=mn2xy(centroid(1),centroid(2));

u=[167+(10*x)*1.2+30, -10*y*1.15 , 25];
%         u=[(1/1.03)*(167+(10*x)), -10*y*1.25, 25];

end
