void nativePinStrength(uint32_t ulPin, boolean value) {
  if (g_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN) return;
  PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].bit.DRVSTR = value ? 1 : 0;
}

void nativeDigitalWrite(uint32_t ulPin, boolean value) {
  if (g_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN) return;
  if (value) PORT->Group[g_APinDescription[ulPin].ulPort].OUTSET.reg = (1 << g_APinDescription[ulPin].ulPin);
  else PORT->Group[g_APinDescription[ulPin].ulPort].OUTCLR.reg = (1 << g_APinDescription[ulPin].ulPin);
}

void debugInit() {
#if defined(DEBUG_CONNECTOR) && DEBUG_CONNECTOR == 1
  PORT->Group[PORTA].PINCFG[31].reg = (uint8_t)(PORT_PINCFG_INEN);
  PORT->Group[PORTA].DIRCLR.reg = (uint32_t)(1 << 31);
  PORT->Group[PORTA].PINCFG[31].reg = (uint8_t)(PORT_PINCFG_INEN);
  PORT->Group[PORTA].DIRSET.reg = (uint32_t)(1 << 31);
  PORT->Group[PORTA].OUTCLR.reg = (1 << 31);

  PORT->Group[PORTA].PINCFG[30].reg = (uint8_t)(PORT_PINCFG_INEN);
  PORT->Group[PORTA].DIRCLR.reg = (uint32_t)(1 << 30);
  PORT->Group[PORTA].PINCFG[30].reg = (uint8_t)(PORT_PINCFG_INEN);
  PORT->Group[PORTA].DIRSET.reg = (uint32_t)(1 << 30);
  PORT->Group[PORTA].OUTCLR.reg = (1 << 30);
#endif
}

void debug0low() {
#if defined(DEBUG_CONNECTOR) && DEBUG_CONNECTOR == 1
  PORT->Group[PORTA].OUTCLR.reg = (1 << 31);
#endif
}

void debug0high() {
#if defined(DEBUG_CONNECTOR) && DEBUG_CONNECTOR == 1
  PORT->Group[PORTA].OUTSET.reg = (1 << 31);
#endif
}

void debug1low() {
#if defined(DEBUG_CONNECTOR) && DEBUG_CONNECTOR == 1
  PORT->Group[PORTA].OUTCLR.reg = (1 << 30);
#endif
}

void debug1high() {
#if defined(DEBUG_CONNECTOR) && DEBUG_CONNECTOR == 1
  PORT->Group[PORTA].OUTSET.reg = (1 << 30);
#endif
}
