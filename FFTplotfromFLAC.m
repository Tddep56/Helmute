% Load FLAC file
[signal, fs] = audioread('ian motorcycle wind noise.flac');  % Replace with your actual file

% Convert to mono if stereo
if size(signal, 2) > 1
    signal = mean(signal, 2);
end

% Parameters
frameSize = 2048;
hopSize = 1024;
numFrames = floor((length(signal) - frameSize) / hopSize) + 1;
fftSize = frameSize;
freqAxis = linspace(0, fs/2, fftSize/2 + 1);

% Initialize
fftSum = zeros(fftSize/2 + 1, 1);

% Process frames
for i = 1:numFrames
    startIdx = (i - 1) * hopSize + 1;
    frame = signal(startIdx:startIdx + frameSize - 1) .* hann(frameSize);
    fftFrame = fft(frame, fftSize);
    mag = abs(fftFrame(1:fftSize/2 + 1));
    fftSum = fftSum + mag;
end

% Average magnitude
fftAvg = fftSum / numFrames;

% Limit to 0-6000 Hz
maxFreq = 5000;
validIdx = freqAxis <= maxFreq;

% Plot
figure('Position', [100, 100, 1200, 400]);  % Wider figure
plot(freqAxis(validIdx), fftAvg(validIdx));
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('Frequency Composition of Motorcycle Noise (Averaged Over 1 Minute)');
grid on;

% Set x-axis ticks every 500 Hz
xticks(0:500:5000);
xlim([0 5000]);