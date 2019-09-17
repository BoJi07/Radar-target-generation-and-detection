clear all
clc;
close all;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz

% Max Range = 200m
% Range Resolution = 1 m

% Max Velocity = 100 m/s

%%%%%%%%%%%%%%%%%%%%%%%%%%%

Range_res = 1;
Light_speed = 3e8;
Max_range = 200;
Max_velocity = 100;


%speed of light = 3e8
%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
Range_target = 120;
Velocity_target = 40; 


%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.


%Operating carrier frequency of Radar 
fc= 77e9;             %carrier freq

B_sweep = Light_speed/(2*Range_res);

T_c = 5.5*2*Max_range/Light_speed;

slope = B_sweep/T_c;
                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*T_c,Nr*Nd); %total time for samples

%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal


%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    
    r_t(i) = Range_target + t(i)*Velocity_target;
    td(i) = r_t(i)*2/Light_speed;
    
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2*pi*(fc*t(i)+0.5*slope*t(i)^2)); 
    Rx(i) = cos(2*pi*(fc*(t(i)-td(i))+0.5*(t(i)-td(i))^2*slope));
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i).*Rx(i);
    
end

%% RANGE MEASUREMENT


 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.

Mix = reshape(Mix,[Nr, Nd]);

 % *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
fft1D = fft(Mix,Nr); 

 % *%TODO* :
% Take the absolute value of FFT output

fft1D = abs(fft1D);
fft1D = fft1D/max(fft1D);

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.

final_fft1D = fft1D(1:Nr/2+1);

%plotting the range
figure ('Name','Range from First FFT')
plot(fft1D)
axis ([0 200 0 1]);
ylabel('Normalized Amplitude')
xlabel('Range');
 % *%TODO* :
 % plot FFT output 

 




% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM
% 
% 
% Range Doppler Map Generation.
% 
% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);

figure ('Name','surface plot of FFT2')
surf(doppler_axis,range_axis,RDM);
title('FFT2 surface plot')
xlabel('speed')
ylabel('Range')
zlabel('Amplitude')

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :
%Select the number of Training Cells in both the dimensions.
Tr = 10;
Tc = 7;

% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gc = 4;
Gr = 4;
% *%TODO* :
% offset the threshold by SNR value in dB
offset = 1.6;
% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells



% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.


   % Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
   % CFAR
   % normalized the RDM
   
    RDM = RDM/max(max(RDM));
    [row,col] = size(RDM);
    CUT = zeros(row,col);
    
    for i = Tr+Gr+1:(Nr/2-Tr-Gr)
        for j = Tc+Gc+1:(Nd-Tc-Gc)
            noise_level = zeros(1,1);
            for k = (i-Tr-Gr) : (i+Tr+Gr)
                for h = (j-Tc-Gc) : (j+Gc+Tc)
                    if(abs(k-i)>Gr||abs(h-j)>Gc)
                        noise_level = noise_level + db2pow(RDM(k,h));
                    end
                end
            end
            length = 2*(Tr+Gr)+1;
            width =  2*(Tc+Gc)+1;
            threshold = pow2db(noise_level/(length*width-(2*Gr+1)*(2*Gc+1)))*1.4;
            if(RDM(i,j)>threshold)
                CUT(i,j) = 1;
            else
                CUT(i,j) = 0;
            end
        end
    end


% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 
 








% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure('Name', 'CA-CFAR Filtered RDM')
surf(doppler_axis,range_axis,CUT);
colorbar;
title( 'CA-CFAR Filtered RDM surface plot');
xlabel('Speed');
ylabel('Range');
zlabel('Normalized Amplitude');

figure('Name', 'CA-CFAR Filtered Range')
surf(doppler_axis,range_axis,CUT);
colorbar;
title( 'CA-CFAR Filtered RDM surface plot');
xlabel('Speed');
ylabel('Range');
zlabel('Normalized Amplitude');
view(90,0);

figure('Name', 'CA-CFAR Filtered Speed')
surf(doppler_axis,range_axis,CUT);
colorbar;
title( 'CA-CFAR Filtered RDM surface plot');
xlabel('Speed');
ylabel('Range');
zlabel('Normalized Amplitude');
view(0,0);
 
 