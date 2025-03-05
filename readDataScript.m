# Das Skript liest in Multiplot gespeicherte Daten wieder ein
# und zeigt sie in Plot-Fenster an
#
dataStream = [];

function filtered_data = median_filter(data, window_size)
  n = length(data);
  half_window = floor(window_size / 2);
  filtered_data = zeros(1,n);
  for i = 1:n
    start_idx = max(1,i - half_window);
    end_idx = min(n, i + half_window);
    filtered_data (i) = median(data(start_idx:end_idx));
  endfor
end
function filtered_data = mean_filter(data, window_size)
  n = length(data);
  half_window = floor(window_size / 2);
  filtered_data = zeros(1,n);
  for i = 1:n
    start_idx = max(1,i - half_window);
    end_idx = min(n, i + half_window);
    filtered_data (i) = mean(data(start_idx:end_idx));
  endfor
end

% Savitzky-Golay Filter Function
function filtered_data = savitzky_golay_filter(data, window_size, poly_order)
   n = length(data);
   half_window = floor(window_size / 2);
   filtered_data = zeros(1, n); % Preallocate filtered data

   % Generate the convolution coefficients
   x = -half_window:half_window;
   A = zeros(window_size, poly_order + 1);
   for i = 0:poly_order
       A(:, i + 1) = x.^i;
   end
   % Compute pseudoinverse of A
   G = pinv(A' * A) * A';

   % Extract the smoothing coefficients (center point)
   coeff = G(1, :);

   % Apply filter
   for i = 1:n
       % Determine the window range
       start_idx = max(1, i - half_window);
       end_idx = min(n, i + half_window);
       window_data = data(start_idx:end_idx);

       % Adjust coefficients for boundary cases
       coeff_start = half_window + 1 - (i -start_idx);
       coeff_end  = coeff_start + length(window_data) - 1;
       coeff_adjusted = coeff(coeff_start:coeff_end);
       filtered_data(i) = sum(window_data .* coeff_adjusted);
   end
end

readData = load("marie_06-02-25.txt");
streamCount = (length(readData.dataMatrix)/3)

for i = 1:streamCount
  dataStream(i).name = readData.dataMatrix{(i-1)*3+1};
  dataStream(i).array = readData.dataMatrix{(i-1)*3+2};
  dataStream(i).t     = readData.dataMatrix{(i-1)*3+3};
endfor

##for i = 1:length(dataStream)
##  plot(dataStream(i).t,dataStream(i).array)
##endfor
daten = dataStream(1).array(1:1000);
plot(daten);

d = mean(daten)
daten = daten-d;

spektrum=fftshift(abs(fft(daten)));
fa=200;
n=1000;
df = fa/n;
f=-fa/2:df:fa/2-df;
figure(2)
plot(f,spektrum)
axis([-100 100 0 2000])

mean_daten = mean_filter(spektrum,8);
figure(3)
plot(f,mean_daten)
median_daten = median_filter(mean_daten,8);
figure(4)
plot(f,median_daten)

##% Apply the Savitzky-Golay filter
##window_size = 15;  % Must be odd
##poly_order = 2;   % Order of the polynomial
##filtered_data = savitzky_golay_filter(daten, window_size, poly_order);
##
##figure(2)
##plot(filtered_data)




