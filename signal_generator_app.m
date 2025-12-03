function signal_generator_app
% =========================================================================
% SIGNAL_GENERATOR_APP FOR SPR307B Project
% =========================================================================
%% Parameters
freqs = [2, 90, 700];
Fs = 20000;
dur = 2.5; % Default duration
t = 0:1/Fs:dur-1/Fs;
%% Create UI
fig = uifigure('Name','SPR307B Signal Generator (Student: 219407297) - V11','Position',[100 100 1100 700]);
% UI Controls
uilabel(fig,'Position',[20 640 150 22],'Text','Select Frequency:');
bgFreq = uibuttongroup(fig,'Position',[20 600 300 50]);
rb1 = uiradiobutton(bgFreq,'Position',[10 10 100 22],'Text',sprintf('%d Hz',freqs(1)),'Value',true);
rb2 = uiradiobutton(bgFreq,'Position',[110 10 120 22],'Text',sprintf('%d Hz',freqs(2)));
rb3 = uiradiobutton(bgFreq,'Position',[230 10 140 22],'Text',sprintf('%d Hz',freqs(3)));

uilabel(fig,'Position',[340 640 120 22],'Text','Amplitude (V):');
sldAmp = uislider(fig,'Position',[340 620 200 3],'Limits',[0 6],'Value',5,'Enable','off');
uilabel(fig,'Position',[340 600 200 22],'Text','(Fixed at 5V per project spec)');

uilabel(fig,'Position',[560 640 120 22],'Text','Fs (Hz):');
edFs = uieditfield(fig,'numeric','Position',[560 610 100 30],'Value',Fs);
uilabel(fig,'Position',[680 640 120 22],'Text','Duration (s):');
edDur = uieditfield(fig,'numeric','Position',[680 610 100 30],'Value',dur);

uilabel(fig,'Position',[800 640 200 22],'Text','LP cutoff multiplier (fc = m * f0):');
edCutMul = uieditfield(fig,'numeric','Position',[800 610 100 30],'Value',1.5);
uilabel(fig,'Position',[920 640 200 22],'Text','Add AWGN SNR (dB):');
edSNR = uieditfield(fig,'numeric','Position',[920 610 100 30],'Value',20);

% --- Button Row ---
uibutton(fig,'push','Position',[20 520 140 40],'Text','Generate Square Wave','ButtonPushedFcn',@onGenerate);
uibutton(fig,'push','Position',[180 520 120 40],'Text','Filter to Sine','ButtonPushedFcn',@onFilterToSine);
uibutton(fig,'push','Position',[320 520 100 40],'Text','Add Noise','ButtonPushedFcn',@onAddNoise);
uibutton(fig,'push','Position',[440 520 100 40],'Text','Denoise','ButtonPushedFcn',@onDenoise);
uibutton(fig,'push','Position',[560 520 100 40],'Text','Send UDP','ButtonPushedFcn',@onSendUDP);
uibutton(fig,'push','Position',[680 520 100 40],'Text','Save WAV','ButtonPushedFcn',@onSaveWAV);
uibutton(fig,'push','Position',[800 520 120 40],'Text','Zoom In (5 Cycles)','ButtonPushedFcn',@onZoomIn);
uibutton(fig,'push','Position',[940 520 100 40],'Text','Zoom Out (Full)','ButtonPushedFcn',@onZoomOut);

% Axes
axTime = uiaxes(fig,'Position',[40 40 500 430]);
xlabel(axTime,'Time (s)'); ylabel(axTime,'Voltage (V)');
axFreq = uiaxes(fig,'Position',[580 40 500 430]);
xlabel(axFreq,'Frequency (Hz)'); ylabel(axFreq,'Single-Sided Amplitude');

% State variables
state.clean = []; % Holds the clean signal (square or sine)
state.noisy = [];
state.denoised = [];
state.is_filtered = false; % Flag to track signal type
state.t = t; state.Fs = Fs; state.freqs = freqs;
state.axTime = axTime; state.axFreq = axFreq;
state.t_plot = t; % Initialize t_plot
fig.UserData = state;
%% Callback functions
    function onGenerate(~,~)
        s = fig.UserData;
        Fs = edFs.Value; dur = edDur.Value;
        t = 0:1/Fs:dur-1/Fs;
        s.t = t; s.Fs = Fs;
        
        f0 = getSelectedFreq();
        A = sldAmp.Value;
        
        % Generate and save the square wave
        sq = A * sign(sin(2*pi*f0*t));
        s.clean = sq;
        
        % Reset other signals and set flag
        s.noisy = [];
        s.denoised = [];
        s.is_filtered = false; 
        
        fig.UserData = s;
        plotAll(s, f0);
    end

    function onFilterToSine(~,~)
        s = fig.UserData;
        if isempty(s.clean), uialert(fig,'Generate a square wave first.','No Signal'); return; end
        if s.is_filtered, uialert(fig,'Signal is already filtered to sine.','Already Sine'); return; end

        f0 = getSelectedFreq();
        Fs = s.Fs;
        m = edCutMul.Value;
        sq = s.clean; % Get the clean square wave
        
        % Apply the filter logic
        if f0 == 2
            m = 2.5;
            Fs_low = 2000;
            sq_low = resample(sq, Fs_low, Fs);
            fc = m * f0;
            Wc = fc / (Fs_low/2);
            [sos, g] = butter(6, Wc, 'low');
            sig_low = filtfilt(sos, g, sq_low);
            sig = resample(sig_low, Fs, Fs_low);
        else
            fc = m * f0;
            Wc = fc/(Fs/2);
        
            if Wc >= 1.0
                warning('Cutoff frequency is too high for the sampling rate. Increase Fs or reduce the multiplier.');
                Wc = 0.99;
            end
        
            [sos, g] = butter(6, Wc, 'low');
            sig = filtfilt(sos, g, sq);
        end
        
        % Save the new clean sine wave
        s.clean = sig;
        s.noisy = [];
        s.denoised = [];
        s.is_filtered = true; % Set flag
        
        fig.UserData = s;
        plotAll(s, f0);
    end

    function onAddNoise(~,~)
        s = fig.UserData;
        if isempty(s.clean), uialert(fig,'Generate a signal first.','No Signal'); return; end
        SNR_dB = edSNR.Value;
        s.noisy = awgn(s.clean, SNR_dB, 'measured'); % Add noise to the current clean signal
        s.denoised = [];
        fig.UserData = s;
        plotAll(s, getSelectedFreq());
    end

    function onDenoise(~,~)
        s = fig.UserData;
        if isempty(s.noisy), uialert(fig,'Add noise to the signal first.','No Noisy Signal'); return; end
        
        f0 = getSelectedFreq();
        Fs = s.Fs;
        m = edCutMul.Value;
        
        % Apply the same filter logic to the *noisy* signal
        if f0 == 2
            m = 2.5;
            Fs_low = 2000;
            noisy_low = resample(s.noisy, Fs_low, Fs);
            fc = m * f0;
            Wc = fc / (Fs_low/2);
            [sos, g] = butter(6, Wc, 'low');
            denoised_low = filtfilt(sos, g, noisy_low);
            s.denoised = resample(denoised_low, Fs, Fs_low);
        else
            fc = m * f0;
            Wc = fc / (Fs/2);
            if Wc >= 1.0
                warning('Cutoff frequency is too high for the sampling rate. Increase Fs or reduce the multiplier.');
                Wc = 0.99; 
            end
        
            [sos, g] = butter(6, Wc, 'low');
            s.denoised = filtfilt(sos, g, s.noisy);
        end
        
        fig.UserData = s;
        plotAll(s, f0);
    end

    function onSendUDP(~,~)
        s = fig.UserData;
        if isempty(s.clean), uialert(fig,'Generate a signal first.','No Signal'); return; end
        
        if ~isempty(s.denoised), vec = s.denoised;
        elseif ~isempty(s.noisy), vec = s.noisy;
        else, vec = s.clean; end
        
        vec_norm = vec / max(abs(vec), [], 'all');
        payload = int16(vec_norm * 32767);
        prompt = {'Receiver IP (e.g., 127.0.0.1):','Receiver Port (e.g., 9000):'};
        dlg = inputdlg(prompt,'UDP Send',1,{'127.0.0.1','9000'});
        if isempty(dlg), return; end
        ip = dlg{1}; port = str2double(dlg{2});
        
        try
            sender = udpsender(ip, port);
            sender(payload);
            uialert(fig,sprintf('Sent %d samples to %s:%d via UDP',length(payload),ip,port),'UDP Send Success');
        catch ME
            uialert(fig,sprintf('UDP send failed: %s',ME.message),'Error');
        end
    end

    function onSaveWAV(~,~)
        s = fig.UserData;
        if isempty(s.clean), uialert(fig,'Generate a signal first.','No signal'); return; end
        
        if ~isempty(s.denoised), out = s.denoised;
        elseif ~isempty(s.noisy), out = s.noisy;
        else, out = s.clean; end
        
        [file,path] = uiputfile('signal.wav','Save signal as WAV');
        if isequal(file,0), return; end
        filename = fullfile(path,file);
        
        outn = out / max(abs(out), [], 'all');
        audiowrite(filename, outn, s.Fs);
        uialert(fig,['Saved as ' filename],'File Saved');
    end

    function onZoomIn(~,~)
        s = fig.UserData;
        if isempty(s.clean), uialert(fig,'Generate a signal first.','No Signal'); return; end
        
        f0 = getSelectedFreq();
        t_plot = s.t_plot; % Get the time vector from the state
        
        num_cycles = 5;
        plot_duration = min(t_plot(end), num_cycles / f0);
        
        xlim(s.axTime, [0, plot_duration]);
    end

    function onZoomOut(~,~)
        s = fig.UserData;
        if isempty(s.clean), uialert(fig,'Generate a signal first.','No Signal'); return; end

        t_plot = s.t_plot;
        
        xlim(s.axTime, [0, t_plot(end)]);
    end
    
%% Utility functions
    function f0 = getSelectedFreq()
        if rb1.Value, f0 = freqs(1);
        elseif rb2.Value, f0 = freqs(2);
        else, f0 = freqs(3); end
    end

    function plotAll(s, f0)
        axT = s.axTime; axF = s.axFreq;
        cla(axT); cla(axF);
        Fs = s.Fs; t = s.t;
        
        % This logic correctly selects the right signal to plot
        if ~isempty(s.denoised)
            sig_plot = s.denoised; legendStr = 'Denoised';
        elseif ~isempty(s.noisy)
            sig_plot = s.noisy; legendStr = 'Noisy';
        else
            sig_plot = s.clean; % Plot the clean signal
            
            % Set legend based on the flag
            if s.is_filtered
                legendStr = 'Clean Sine';
            else
                legendStr = 'Clean Square';
            end
        end

        if length(t) ~= length(sig_plot)
             t_plot = (0:length(sig_plot)-1) / Fs;
        else
             t_plot = t;
        end
        
        fig.UserData.t_plot = t_plot; 

        % Time Domain Plot (Full Signal)
        plot(axT, t_plot, sig_plot);
        title(axT, sprintf('Time Domain: %s Signal (%g Hz)', legendStr, f0));
        grid(axT,'on'); legend(axT, legendStr);
        xlim(axT, [0, t_plot(end)]); % Default to zoomed out

        % Frequency Domain Plot (FFT)
        N = length(sig_plot);
        P2 = abs(fft(sig_plot)/N);
        P1 = P2(1:floor(N/2)+1);
        P1(2:end-1) = 2*P1(2:end-1);
        f = Fs*(0:(N/2))/N;
        
        plot(axF, f, P1);
        title(axF, 'Frequency Domain (Single-Sided Amplitude Spectrum)');
        grid(axF,'on');
        
        % **FIX: Smarter Dynamic X-axis for FFT plot**
        if strcmp(legendStr, 'Clean Square') || strcmp(legendStr, 'Clean Sine')
            % For clean signals, zoom in on harmonics
            xlim_clean = min(15 * f0, Fs/2);
            % Ensure a minimum visible range for 2Hz sine wave
            if f0 == 2 && s.is_filtered % Use the flag
                xlim_clean = max(xlim_clean, 10); % Show at least 0-10 Hz
            end
            xlim(axF, [0, xlim_clean]);
        else
            % For Noisy/Denoised, show the "action area" of the filter
            if f0 == 2
                xlim(axF, [0, 20]); % Show 0-20 Hz
            elseif f0 == 90
                xlim(axF, [0, 450]); % Show 0-450 Hz (up to 5th harmonic)
            else % f0 == 700
                xlim(axF, [0, 3500]); % Show 0-3500 Hz (up to 5th harmonic)
            end
        end
    end
end
