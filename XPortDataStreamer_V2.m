#          XPortDataStreamer V 2.0  (XPDS)
#  (c) Jens Bongartz, 2025, Hochschule Koblenz
#                Stand: 05.03.2025
#  ================================================
pkg load instrument-control;
pkg load signal;

clear all; clc;
#profile on
#inputDevice = "serial";
inputDevice = "udp";

# obj = dataStreamClass(name,plcolor,dt,plotwidth,plot,filter)
dataStream(1) = dataStreamClass("FBT","red",800,1,1); # externe Klasse
# createIIRFilter(f_abtast,f_HP,f_NO,f_TP)
dataStream(1).createIIRFilter(200,2,50,40);

if strcmp(inputDevice,"serial")
   baudrate = 115200;
   inputPort = serialPortClass(baudrate);
endif

if strcmp(inputDevice,"udp")
   inputPort = udpPortClass();                   # externe Klasse
endif

inputPort.createSelector(dataStream);         # >> inputPort.streamSelector
inputPort.createRegEx(dataStream);            # >> inputPort.regex_pattern

# Globale Variablen zur Programmsteuerung
# fuer Test mit FIR-Filter sind alle IIR-Filter zunaechst deaktiviert
global HP_filtered = 1 NO_filtered = 1 TP_filtered = 1 AM_filtered = 1;
# und FIR-Filter ist aktiviert
global FIR_filtered = 0;

global quit_prg = 0 clear_data = 0 save_data = 0 rec_data = 1;

# Einstellwerte fuer die Programm-Performance [Sekunden]
Bench_Time   = 2;       # measurment of load every 2 seconds
Plot_Time    = 0.03;
Port_Time    = 0.03;    # alle 30ms (33 Hz) werden Daten empfangen
Pause_Time   = 0.02;
#Pause_Time   = 0.0;

# Der weitere Teil wird nur ausgefuehrt, wenn serielle Schnittstelle gefunden wurde
if !isempty(inputPort)
  # Graphikfenster initialisieren
  plotGraph = plotGraphClass(dataStream);       # externe Klasse
  cap = GUI_Elements(plotGraph.fi_1);           # externe Funktion
  displayInfo(plotGraph.fi_1);                  # externe Funktion
  inputPort.clearPort();                        # externe Klasse
  # Hauptschleife
  # =============
  datasetCounter = 0; datasetCounter_tic = 0; bytesReceived = 0;

  bench_tic = tic(); plot_tic = tic(); port_tic = tic();
  [t_cpu_prev,t_user_prev,t_sys_prev] = cputime();

  do
    ## Clear-Button
    if (clear_data)
      j = 0;
      for i = 1:length(dataStream);
        dataStream(i).clear;
        if (dataStream(i).plot > 0)
          j = j + 1;
          set(plotGraph.subPl(j),"xlim",[0 dataStream(i).plotwidth*5]);
        endif
      endfor
      datasetCounter = 0;  datasetCounter_prev = 0;
      clear_data = 0;
    endif
    ## Save-Button
    if (save_data)
      rec_data = 0;
      dataMatrix = {};
      for i = 1:length(dataStream)
        ar_index = dataStream(i).ar_index;
        dataMatrix{end+1} = dataStream(i).name;
        dataMatrix{end+1} = [dataStream(i).array(ar_index:end) dataStream(i).array(1:ar_index-1)];
        dataMatrix{end+1} = [dataStream(i).t(ar_index:end) dataStream(i).t(1:ar_index-1)];
      endfor
      myfilename = uiputfile();
      if (myfilename != 0)
        save("-text",myfilename,"dataMatrix");
      endif
      save_data = 0;
    endif

    # Port auslesen
    # =============
    po_toc = toc(port_tic);
    if (po_toc > Port_Time)
      [bytesAvailable,inChar] = inputPort.readPort();  # >> inputPort.inBuffer;
      bytesReceived = bytesReceived + bytesAvailable;
      if (rec_data)   # Wird vom REC-Button gesteuert
        if (length(inChar) > 0)
           countMatches = inputPort.parseInput(inChar,dataStream);
           datasetCounter = datasetCounter + countMatches;         # datasetCounter laeuft durch
           datasetCounter_tic = datasetCounter_tic + countMatches; # datasetCounter_tic >> Bench_Time
        endif
      endif
      port_tic = tic();
    endif # po_toc

    # Plot-Graphikfenster
    # ===================
    p_toc = toc(plot_tic);
    if (p_toc > Plot_Time)
      plotGraph.draw(dataStream);
      if (ishandle(plotGraph.fi_1))
        set(cap(1),"string",num2str(datasetCounter));
      endif
      drawnow();
      plot_tic = tic();
    endif # p_toc
    # Entlastung der CPU
    pause(Pause_Time);

    # Benchmarking
    # ============
    b_toc = toc(bench_tic);
    if (b_toc > Bench_Time)
      # Abtastrate geschaetzt
      f_abtast = round(datasetCounter_tic/b_toc);
      datasetCounter_tic = 0;
      bytesPerSecond = round(bytesReceived / b_toc);
      bytesReceived = 0;

      [t_cpu,t_user,t_sys] = cputime();
      user_load = t_user - t_user_prev;
      sys_load  = t_sys - t_sys_prev;
      t_cpu_prev = t_cpu;
      t_user_prev = t_user;
      t_sys_prev = t_sys;
      if (ishandle(plotGraph.fi_1))
        set(cap(2),"string",num2str(f_abtast));
#        set(cap(3),"string",num2str(b_toc,"%.2f"));
        set(cap(3),"string",num2str(sys_load,"%.2f"));
        set(cap(4),"string",num2str(user_load,"%.2f"));
        set(cap(5),"string",num2str(bytesPerSecond));
        set(cap(6),"string",num2str(countMatches));
      endif # ishandle(fi_1)
      bench_tic=tic();
    endif # b_toc
  until(quit_prg);
  clear inputPort;
endif
profile off

