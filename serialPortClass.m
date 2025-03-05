classdef serialPortClass < handle

  properties
    streamSelector = [];
    regex_pattern = '';
    serialPortPath = '';
    inBuffer = '';
    port_01 = '';
  endproperties

  methods
    function self = serialPortClass(baudrate)    # Constructor
      close self.port_01;
      disp('Searching Serial Port ... ')
      i = 0;
      do
        i = i + 1;
        disp(i)
        self.serialPortPath = self.checkSerialPort(baudrate);
      until (!isempty(self.serialPortPath) || i == 3)
      if (!isempty(self.serialPortPath))
        disp("Serial Port found:")
        disp(self.serialPortPath)
      else
        disp("No Device found!");
      endif
      if (!isempty(self.serialPortPath))
        self.clearPort();
        disp('Receiving data!')
      endif
    endfunction

    function clearPort(self)
      #flush(self.port_01);
      posLF = 0;
      do
        bytesAvailable = self.port_01.NumBytesAvailable;
        if (bytesAvailable > 0)
          inSerialPort = char(read(self.port_01,bytesAvailable));
          posLF        = index(inSerialPort,char(10),"last");
        endif
      until (posLF > 0);
      # erst ab dem letzten \n geht es los
      self.inBuffer = inSerialPort(posLF+1:end);
    endfunction

    function portReturn = checkSerialPort(self,baudrate)
      fehler = false;
      ports = serialportlist();
      portIndex = 1;
      port_found = false;
      portReturn = '';
      while(portIndex <= length(ports) && !port_found)
        #disp(ports{portIndex})
        try
          clear self.port_01;
          disp(ports{portIndex});
          self.port_01 = serialport(ports{portIndex},baudrate);
        catch
          fehler = true;
          disp(lasterror.message);
        end_try_catch
        if (fehler == false)
          #pause(1)
          #flush(port_01);
          pause(2)
          bytesAvailable = self.port_01.NumBytesAvailable;
          if (bytesAvailable > 0)
            inSerialPort = char(read(self.port_01,bytesAvailable));
            firstCRLF    = index(inSerialPort, "\r\n","first");
            lastCRLF     = index(inSerialPort, "\r\n","last");
            if (lastCRLF > firstCRLF)
              inChar   = inSerialPort(firstCRLF:lastCRLF);
              try
                 values   = strsplit(inChar, {':',',','\n','\r'});
              catch
                 disp(lasterror.message);
                 values = {};
              end_try_catch
              data = unique(values);
              filtered_data = {};
              for i = 1:numel(data)
                if !any(isstrprop(data{i}, 'digit'))
                  if !isempty(data{i})
                    filtered_data{end+1} = data{i};
                  endif
                endif
              endfor
              if !isempty(filtered_data)
                msg = [self.port_01.Port ,"\n"];
                for i = 1:length(filtered_data)
                  msg = [msg,filtered_data{i},";"];
                endfor
                portReturn = self.port_01.Port;
                port_found = true;
                disp(msg);
              endif
              #disp(filtered_data)
            endif
          endif                       # bytesAvailable
          # clear port_01;
        else                          # fehler == false
          fehler = false;
        endif
        portIndex = portIndex + 1;
      endwhile
    endfunction

    function [bytesAvailable,inChar] = readPort(self)
      buffer_count = 0;
      do
         buffer_count += 1
         bytesAvailable = self.port_01.NumBytesAvailable
         inSerialPort   = char(read(self.port_01,bytesAvailable));
         self.inBuffer  = [self.inBuffer inSerialPort];
      until (self.port_01.NumBytesAvailable == 0)
      posLF          = index(self.inBuffer,char(10),"last");
      inChar         = '';
      if (posLF > 0)
        inChar   = self.inBuffer(1:posLF);
        self.inBuffer = self.inBuffer(posLF+1:end);
      endif
    endfunction

    function countMatches = parseInput(self,inChar,dataStream)
      matches = regexp(inChar, self.regex_pattern, 'tokens'); # Regular Expression auswerten
      countMatches   = length(matches);                       # Wert wird ausgegeben
      for i = 1:countMatches
        streamName = matches{i}{1};
        adc        = str2num(matches{i}{2});
        sample_t   = str2num(matches{i}{3});
        j = self.streamSelector(streamName);     # Sample einem dataStream zuweisen
        dataStream(j).addSample(adc,sample_t);   # Hier uebernimmt dataStream die Arbeit
      endfor
    endfunction

    function createRegEx(self,dataStream)
      self.regex_pattern = '(';
      for i = 1:length(dataStream)
        self.regex_pattern = [self.regex_pattern dataStream(i).name];
        if i < length(dataStream)
          self.regex_pattern = [self.regex_pattern '|'];
        endif
      endfor
      self.regex_pattern = [self.regex_pattern '):(-?\d+),t:(\d+)'];
    endfunction

    function createSelector(self,dataStream)
      # Liste aller dataStream Namen erstellen fuer Dictonary
      namelist = {};
      for i = 1:length(dataStream)
        namelist{end+1} = dataStream(i).name;
      endfor
      values = 1:numel(dataStream);
      self.streamSelector = containers.Map(namelist,values);
    endfunction
  endmethods
end
