classdef udpPortClass < handle

  properties
    streamSelector = [];
    regex_pattern = '';
    inBuffer = '';
    port_01 = '';
  endproperties

  methods
    function self = udpPortClass()    # Constructor
      close self.port_01;
      disp('Opening UDP Port ... ')
      #udpport gibt Parameter aus
      self.port_01 = udpport ("LocalHost","192.168.1.2","LocalPort",8266);
    endfunction

    function clearPort(self)
      posLF = 0;
      do
        bytesAvailable = self.port_01.NumBytesAvailable;
        if (bytesAvailable > 0)
          inUDPPort = char(read(self.port_01,bytesAvailable));
          posLF     = index(inUDPPort,char(10),"last");
        endif
      until (posLF > 0);
      # erst ab dem letzten \n geht es los
      self.inBuffer = inUDPPort(posLF+1:end);
    endfunction

    function [bytesAvailable,inChar] = readPort(self)
      do
         bytesAvailable = self.port_01.NumBytesAvailable;
         inUDPPort   = char(read(self.port_01,bytesAvailable));
         self.inBuffer  = [self.inBuffer inUDPPort];
      until (self.port_01.NumBytesAvailable == 0)
      posLF          = index(self.inBuffer,char(10),"last");
      inChar         = '';
      if (posLF > 0)
        inChar   = self.inBuffer(1:posLF);
        self.inBuffer = self.inBuffer(posLF+1:end);
      endif
    endfunction

    function countMatches = parseInput(self,inChar,dataStream)
      matches = regexp(inChar, self.regex_pattern, 'tokens');     # Regular Expression auswerten
      countMatches   = length(matches);                           # Wert wird ausgegeben
      if (countMatches == 0)
        disp("RegEx-Error");
        disp(length(inChar));
      endif
      # Code-Optimierung
      # ================
      if countMatches > 0
        [streamNames, sampleCells, timestampCells] = cellfun(@(x) deal(x{1}, x{2}, x{3}),matches,'UniformOutput',false);
        samples = str2double(sampleCells);
        timestamps = str2double(timestampCells);
        #j_indices = cellfun(@(x) self.streamSelector(x),streamNames);
        # Hier werden die Daten für die einzelnen dataStreams vorsortiert.
        # Jeder Datenstrom wird dann nur einmal mit einer Liste aufgerufen.
        % Erzeuge eine Lookup-Tabelle für eindeutige Stream-Namen
        uniqueStreams = unique(streamNames);
        % Gruppiere die Daten nach Streams
        groupedData = struct();
        for i = 1:length(uniqueStreams)
          stream = uniqueStreams{i};
          idx = strcmp(streamNames,stream);
          % Speichere gruppierte Daten in der Struktur
          groupedData.(stream).samples = samples(idx);
          groupedData.(stream).timestamps = timestamps(idx);
        endfor
        % Aufruf für jeden Stream nur einmal
        for k = 1:length(uniqueStreams)
          j = self.streamSelector(stream);
          % Übergabe der gesamten Liste auf einmal an addSample
          dataStream(j).addSamples(groupedData.(stream).samples,groupedData.(stream).timestamps);
        endfor
      endif
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
