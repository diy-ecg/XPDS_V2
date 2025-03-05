function displayInfo(fi_1)
  #clc
  disp("Computer");
  disp("======== ");
  disp(uname().sysname)
  disp(uname().nodename)
  disp(uname().release)
  disp(uname().version)
  disp(uname().machine)
  disp(" ")
  ver_struct = ver();
  disp("Octave");
  disp("====== ");
  disp(ver_struct(1).Version)
  %disp(ver_struct(1).Release)
  disp(ver_struct(1).Date)
  disp("instrument-control");
  disp("==================");
  for i = 2:length(ver_struct)
    if strcmp(ver_struct(i).Name, 'instrument-control')
      disp(ver_struct(i).Version)
      %disp(ver_struct(i).Release)
      disp(ver_struct(i).Date)
    endif
  endfor
  % Graphics
  % ========
  disp("Graphics Properties")
  disp("===================");
  disp(strcat("gl_renderer  :",get(fi_1,"__gl_renderer__")))
  disp(strcat("gl_vendor    :",get(fi_1,"__gl_vendor__")))
  disp(strcat("gl_version   :",get(fi_1,"__gl_version__")))
  disp(strcat("Toolkit      :",get(fi_1,"__graphics_toolkit__")))
  disp(strcat("Renderer     :",get(fi_1,"renderer")))
  disp(strcat("Renderer-Mode:",get(fi_1,"renderermode")))


##  disp("Available COM-Ports");
##  disp("===================");
##  ports = serialportlist();
##  if (length(ports) > 0)
##     for i = 1:length(ports)
##       disp(ports{i})
##     endfor
##  endif
##  disp(" ")
endfunction
