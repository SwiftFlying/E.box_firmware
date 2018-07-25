void nativePinStrength(uint32_t ulPin, boolean value) {
  if (g_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN) return;
  PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].bit.DRVSTR = value ? 1 : 0;
}

void nativeDigitalWrite(uint32_t ulPin, boolean value) {
  if (g_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN) return;
  if (value) PORT->Group[g_APinDescription[ulPin].ulPort].OUTSET.reg = (1 << g_APinDescription[ulPin].ulPin);
  else PORT->Group[g_APinDescription[ulPin].ulPort].OUTCLR.reg = (1 << g_APinDescription[ulPin].ulPin);
}

