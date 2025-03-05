# dataStreamClass.m >> This is a handle-Class!
classdef dataStreamClass < handle

    properties
        # ar_index >> Counter fuer Ringspeicher
        # index    >> Counter fuer alle Abtastwerte des dataStream
        name      = "";
        array     = []; ar_index  = 1; length = 2000; index = 1;
        t         = [];
        plotwidth = 800; plot     = 1; plcolor   = "";
        #ylim      = 0;             #yalternativ lim = [0 100];
        ylim      = [-100 150];    #alternativ lim = [0 100];
        filter    = 1;

        HP_nr_ko = [0 0 0];
        HP_re_ko = [1 0 0];
        HP_filt_sp = [0 0];

        TP_nr_ko = [0 0 0];
        TP_re_ko = [1 0 0];
        TP_filt_sp = [0 0];

        NO_nr_ko = [0 0 0];
        NO_re_ko = [1 0 0];
        NO_filt_sp = [0 0];

        # Echtzeit-Signalanalyse
        fs = 200; % Abtastrate in Hz
        window_size      = 0;
        inhibit_time     = 0;
        max_window_size  = 0;
        dynamic_threshold = 0;
        rr_interval      = 0;

        buffer           = [];
        buffer_index     = 1;
        sum_val          = 0;
        filter_disable   = false;
        inhibit_counter  = 0;
        last_r_peak_time = 0;
        prev_r_peak_time = 0;
        bpm_values       = [];
        max_buffer       = [];
        max_index        = 1;

        BPM = 0;
        newBMP = 0;
    endproperties

    methods (Access=public)

        function self = dataStreamClass(name,plcolor,plotwidth,plot,filter)
          self.name      = name;
          self.plcolor   = plcolor;
          self.plotwidth = plotwidth;
          self.plot      = plot;
          self.filter    = filter;
          self.initRingBuffer();

          self.fs = 200; % Abtastrate in Hz
          self.window_size = round(0.2 * self.fs); % Gleitender Mittelwert über 200 ms
          self.inhibit_time = round(0.05 * self.fs); % Filteraussetzung für 50 ms nach R-Zacke
          self.max_window_size = round(2 * self.fs); % Fenstergröße für das iterative Maximum über die letzten 2 Sekunden

          self.buffer = zeros(self.window_size, 1);
          self.max_buffer = zeros(self.max_window_size, 1);
        endfunction

        function initRingBuffer(self)
          self.array    = zeros(1,self.length);
          self.t        = zeros(1,self.length);
          self.index    = 1;
          self.ar_index = 1;
        endfunction

        function createIIRFilter(self,f_abtast,f_HP,f_NO,f_TP)
          fa_2 = f_abtast / 2;
          # Tiefpass-IIR-Filter
          [self.TP_nr_ko, self.TP_re_ko] = butter(2,f_TP/fa_2,"low");
          # Hochpass-IIR-Filter
          [self.HP_nr_ko, self.HP_re_ko] = butter(2,f_HP/fa_2,"high");
          # Notch-IIR Filter
          [self.NO_nr_ko, self.NO_re_ko] = pei_tseng_notch(f_NO/fa_2, 2/fa_2);
        endfunction

        function createFIRFilter(self,fa,df,f1,f2)
          # Erstellen der Filterkoeffizienten
          self.FIR_ko = calcFIRCoeff(fa,df,f1,f2);
          # Erstellen des FIR-Eingangsbuffer entsprechend der Filtergroesse
          self.FIR_sp = zeros(1,length(self.FIR_ko));
          # beide Vektoren sind persistent mit dem dataStream Objekt
        endfunction

        function addSamples(self,samples,timestamps)
          global HP_filtered NO_filtered TP_filtered AM_filtered FIR_filtered;
          std_sig = std(samples);
          #disp(std_sig);
          #if (std_sig < 100)   # Signal prüfen

          if (NO_filtered)
            [samples, self.NO_filt_sp] = filter(self.NO_nr_ko, self.NO_re_ko,samples,self.NO_filt_sp);
          endif
          if (TP_filtered)
            [samples, self.TP_filt_sp] = filter(self.TP_nr_ko, self.TP_re_ko,samples,self.TP_filt_sp);
          endif
          if (HP_filtered)
            [samples, self.HP_filt_sp] = filter(self.HP_nr_ko, self.HP_re_ko,samples,self.HP_filt_sp);
          endif

          samples(abs(samples)<0.001)=0;

          for k = 1:length(samples)

            sample = samples(k);
            timestamp = timestamps(k);

            if (AM_filtered)
            # Beginn der Echtzeit Signalanalyse

            % Dynamische Berechnung des Schwellenwerts über die letzten 2 Sekunden
            self.max_buffer(self.max_index) = sample;
            self.max_index = mod(self.max_index, self.max_window_size) + 1;
            self.dynamic_threshold = 0.3 * max(self.max_buffer); % Adaptives Maximum über die letzten 2 Sekunden
            % R-Zacken-Erkennung
            if sample > self.dynamic_threshold && (timestamp - self.last_r_peak_time) > 300 % Mindestens 300ms Abstand
              self.prev_r_peak_time = self.last_r_peak_time;
              self.last_r_peak_time = timestamp;
               % BPM-Berechnung
              if self.prev_r_peak_time > 0
                 self.rr_interval = self.last_r_peak_time - self.prev_r_peak_time; % RR-Intervall in Sekunden
                 self.BPM = round(60000 / self.rr_interval);
                 self.newBMP = 1;
                 #disp(self.BPM);
              endif
              % Mittelwertfilter temporär deaktivieren
              self.filter_disable = true;
              self.inhibit_counter = self.inhibit_time;
            endif
            % Timer für die Inhibit-Phase
            if self.filter_disable && self.inhibit_counter > 0
              self.inhibit_counter = self.inhibit_counter - 1;
            else
              self.filter_disable = false;
            endif

            % Adaptive Mittelwertfilterung
            if self.filter_disable
              sample = sample; % Kein Filter während Inhibit-Phase
            else
              self.sum_val = self.sum_val - self.buffer(self.buffer_index) + sample;
              self.buffer(self.buffer_index) = sample;
              sample = self.sum_val / self.window_size;
              self.buffer_index = mod(self.buffer_index, self.window_size) + 1;
            endif

            endif # if (AM_filtered)
            # Ende der Echtzeit Signalanalyse

            # Hier werden die Daten in den Ringspeicher geschrieben
            self.array(self.ar_index)  = sample;
            self.t(self.ar_index)      = timestamp;

            self.index = self.index + 1;
            # Ringspeicher Indexing
            self.ar_index = self.ar_index + 1;
            if (self.ar_index > self.length)
              self.ar_index = 1;
            endif
          endfor
          #endif     # if (std_sig < 100)
        endfunction

        # Returns the n last samples >> draw-Routine
        function [ret_array, ret_time] = lastSamples(self,n)
          if (self.ar_index - n > 0)          # kein Wrap-Around notwendig
            ret_array = self.array(self.ar_index-n:self.ar_index-1);
            ret_time  = self.t(self.ar_index-n:self.ar_index-1);
          else                                 # n > ar_index >> Wrap-Around
            n1 = n - self.ar_index;
            ret_array = self.array(self.length-n1:self.length);
            ret_array = [ret_array self.array(1:self.ar_index-1)];
            ret_time = self.t(self.length-n1:self.length);
            ret_time = [ret_time self.t(1:self.ar_index-1)];
          endif
        endfunction

        function clear(self)
          self.initRingBuffer();
        endfunction

        function disp(self)
            disp("dataStreamClass");
            disp(self.name);
        endfunction
    endmethods
end
