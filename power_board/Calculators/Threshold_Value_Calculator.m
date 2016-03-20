%
clc; clear
ADC_nbit = 12;
top_val = 2^ADC_nbit - 1;

fprintf(['Since the ADC has %d bits, it''s maximum (unsigned) value is\n',...
        '0x%04X (%d in base10).\n\n'], ADC_nbit, top_val, top_val)
    
Vref = 3.3/1.6;
R1 = 2e3;
R2 = 27e3;
fcn_Vin = @(Vout) Vout * R1 / (R1+R2);

Vthresh = 17;
Vin = fcn_Vin(Vthresh);
fprintf(['Let''s set our power-up threshold at %gV. This will present\n',...
        '%.3gV to the micro''s ADC.\n\n'], Vthresh, Vin);
    
if (Vin > Vref)
    error('ERROR: Your input voltage surpasses the micro''s %gV reference.\n',...
         Vref)
end

deltaV = Vref * 0.05;
res = round( (Vin + deltaV)/Vref * (top_val + 1) );

fprintf(['Therefore, your code should use 0x%04X\n',...
         '(%d in base10) as your threshold value.\n\n'], res, res);
     
errp = res;
res2BigVoltage = @(res_val) ( res_val/(top_val +1) * Vref - deltaV ) * (R1 + R2) / R1;
verrp = res2BigVoltage(errp);

LSB = res2BigVoltage(errp+1) - res2BigVoltage(errp);

fprintf(['Theoretically, you''ll only be %.3gmV off from the %gV target\n',...
    'voltage you wanted. Also, from the perspective of the power bus,\n',...
    'your LSB is %.2gmV\n\n'], (verrp-Vthresh)*1000, Vthresh, LSB*1000)