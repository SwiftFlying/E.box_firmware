void writeVoid(Stream* stream, b64_crc32_coder* encoder, void (*preTransmitAction)(), void (*postTransmitAction)(), int8_t pinTxEn, uint8_t from, uint8_t to) {
  if (preTransmitAction) (*preTransmitAction)();
  initCoder(encoder);

  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x01);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, from);
  updateEncoderStream(stream, encoder, pinTxEn, true, to);

  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);

  finalizeEncoderStream(stream, encoder, pinTxEn);
  if (postTransmitAction) (*postTransmitAction)();
}

void writeBoardInfo(Stream* stream, b64_crc32_coder* encoder, void (*preTransmitAction)(), void (*postTransmitAction)(), int8_t pinTxEn, uint8_t from, uint8_t to) {
  if (preTransmitAction) (*preTransmitAction)();
  initCoder(encoder);

  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x01);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, from);
  updateEncoderStream(stream, encoder, pinTxEn, true, to);

  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x15);

  updateEncoderStream(stream, encoder, pinTxEn, true, hasCan);
  updateEncoderStream(stream, encoder, pinTxEn, true, hasWifi);

  updateEncoderStream(stream, encoder, pinTxEn, true, FIRMWARE_VERSION[0]);
  updateEncoderStream(stream, encoder, pinTxEn, true, FIRMWARE_VERSION[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, FIRMWARE_VERSION[2]);

  uint8_t* u = (uint8_t*)0x0080A00C;
  updateEncoderStream(stream, encoder, pinTxEn, true, *(u + 3));
  updateEncoderStream(stream, encoder, pinTxEn, true, *(u + 2));
  updateEncoderStream(stream, encoder, pinTxEn, true, *(u + 1));
  updateEncoderStream(stream, encoder, pinTxEn, true, *u);

  u = (uint8_t*)0x0080A040;
  updateEncoderStream(stream, encoder, pinTxEn, true, *(u + 3));
  updateEncoderStream(stream, encoder, pinTxEn, true, *(u + 2));
  updateEncoderStream(stream, encoder, pinTxEn, true, *(u + 1));
  updateEncoderStream(stream, encoder, pinTxEn, true, *u);

  u = (uint8_t*)0x0080A044;
  updateEncoderStream(stream, encoder, pinTxEn, true, *(u + 3));
  updateEncoderStream(stream, encoder, pinTxEn, true, *(u + 2));
  updateEncoderStream(stream, encoder, pinTxEn, true, *(u + 1));
  updateEncoderStream(stream, encoder, pinTxEn, true, *u);

  u = (uint8_t*)0x0080A048;
  updateEncoderStream(stream, encoder, pinTxEn, true, *(u + 3));
  updateEncoderStream(stream, encoder, pinTxEn, true, *(u + 2));
  updateEncoderStream(stream, encoder, pinTxEn, true, *(u + 1));
  updateEncoderStream(stream, encoder, pinTxEn, true, *u);

  finalizeEncoderStream(stream, encoder, pinTxEn);
  if (postTransmitAction) (*postTransmitAction)();
}

void writeSettings(Stream* stream, b64_crc32_coder* encoder, void (*preTransmitAction)(), void (*postTransmitAction)(), int8_t pinTxEn, uint8_t from, uint8_t to, settings* sett) {
  if (preTransmitAction) (*preTransmitAction)();
  initCoder(encoder);

  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x01);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, from);
  updateEncoderStream(stream, encoder, pinTxEn, true, to);

  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x72);

  updateEncoderStream(stream, encoder, pinTxEn, true, sett->valid);
  updateEncoderStream(stream, encoder, pinTxEn, true, sett->automaticPowerOn);
  updateEncoderStream(stream, encoder, pinTxEn, true, sett->canID);
  for (uint8_t k = 0; k < sizeof(sett->wifiIP); k++) updateEncoderStream(stream, encoder, pinTxEn, true, sett->wifiIP[k]);
  for (uint8_t k = 0; k < sizeof(sett->wifiGateway); k++) updateEncoderStream(stream, encoder, pinTxEn, true, sett->wifiGateway[k]);
  for (uint8_t k = 0; k < sizeof(sett->wifiSubnet); k++) updateEncoderStream(stream, encoder, pinTxEn, true, sett->wifiSubnet[k]);
  for (uint8_t k = 0; k < sizeof(sett->wifiName); k++) updateEncoderStream(stream, encoder, pinTxEn, true, sett->wifiName[k]);
  for (uint8_t k = 0; k < sizeof(sett->wifiPassword); k++) updateEncoderStream(stream, encoder, pinTxEn, true, sett->wifiPassword[k]);
  for (uint8_t k = 0; k < sizeof(sett->wifiLoginPassword); k++) updateEncoderStream(stream, encoder, pinTxEn, true, sett->wifiLoginPassword[k]);

  finalizeEncoderStream(stream, encoder, pinTxEn);
  if (postTransmitAction) (*postTransmitAction)();
}

void writeBoolean(Stream* stream, b64_crc32_coder* encoder, void (*preTransmitAction)(), void (*postTransmitAction)(), int8_t pinTxEn, uint8_t from, uint8_t to, boolean value) {
  if (preTransmitAction) (*preTransmitAction)();
  initCoder(encoder);

  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x01);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, from);
  updateEncoderStream(stream, encoder, pinTxEn, true, to);

  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x01);

  updateEncoderStream(stream, encoder, pinTxEn, true, value);

  finalizeEncoderStream(stream, encoder, pinTxEn);
  if (postTransmitAction) (*postTransmitAction)();
}

void writeStatus(Stream* stream, b64_crc32_coder* encoder, void (*preTransmitAction)(), void (*postTransmitAction)(), int8_t pinTxEn, uint8_t from, uint8_t to) {
  if (preTransmitAction) (*preTransmitAction)();
  initCoder(encoder);

  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x01);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, from);
  updateEncoderStream(stream, encoder, pinTxEn, true, to);

  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x02);

  updateEncoderStream(stream, encoder, pinTxEn, true, relay);
  updateEncoderStream(stream, encoder, pinTxEn, true, overloadTimer > 0);

  finalizeEncoderStream(stream, encoder, pinTxEn);
  if (postTransmitAction) (*postTransmitAction)();
}

void writeStatusMeasures(Stream* stream, b64_crc32_coder* encoder, void (*preTransmitAction)(), void (*postTransmitAction)(), int8_t pinTxEn, uint8_t from, uint8_t to, volatile samplesBuffer* data) {
  if (preTransmitAction) (*preTransmitAction)();
  initCoder(encoder);

  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x01);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, from);
  updateEncoderStream(stream, encoder, pinTxEn, true, to);

  uint32Value len { .value = 46 };
  updateEncoderStream(stream, encoder, pinTxEn, true, len.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, len.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, len.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, len.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, relay);
  updateEncoderStream(stream, encoder, pinTxEn, true, overloadTimer > 0);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->iRms.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->iRms.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->iRms.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->iRms.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->v1Rms.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->v1Rms.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->v1Rms.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->v1Rms.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->v2Rms.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->v2Rms.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->v2Rms.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->v2Rms.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->hz.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->hz.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->hz.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->hz.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->w.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->w.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->w.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->w.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->va.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->va.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->va.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->va.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->var.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->var.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->var.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->var.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->powerFactor.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->powerFactor.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->powerFactor.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->powerFactor.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, kwh.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kwh.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kwh.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kwh.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, kvah.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kvah.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kvah.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kvah.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, kvarh.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kvarh.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kvarh.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kvarh.bytes[0]);

  finalizeEncoderStream(stream, encoder, pinTxEn);
  if (postTransmitAction) (*postTransmitAction)();
}

void writeStatusMeasuresWaveform(Stream* stream, b64_crc32_coder* encoder, void (*preTransmitAction)(), void (*postTransmitAction)(), int8_t pinTxEn, uint8_t from, uint8_t to, volatile samplesBuffer* data) {
  if (preTransmitAction) (*preTransmitAction)();
  initCoder(encoder);

  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x01);
  updateEncoderStream(stream, encoder, pinTxEn, true, (uint8_t)0x00);
  updateEncoderStream(stream, encoder, pinTxEn, true, from);
  updateEncoderStream(stream, encoder, pinTxEn, true, to);

  uint32Value len { .value = 55 + data->sampleCount.value * 12 };
  updateEncoderStream(stream, encoder, pinTxEn, true, len.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, len.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, len.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, len.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, relay);
  updateEncoderStream(stream, encoder, pinTxEn, true, overloadTimer > 0);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->iRms.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->iRms.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->iRms.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->iRms.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->v1Rms.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->v1Rms.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->v1Rms.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->v1Rms.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->v2Rms.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->v2Rms.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->v2Rms.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->v2Rms.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->hz.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->hz.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->hz.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->hz.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->w.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->w.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->w.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->w.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->va.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->va.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->va.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->va.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->var.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->var.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->var.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->var.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->powerFactor.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->powerFactor.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->powerFactor.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->powerFactor.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, kwh.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kwh.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kwh.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kwh.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, kvah.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kvah.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kvah.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kvah.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, kvarh.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kvarh.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kvarh.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, kvarh.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->overload);

  floatValue f = { .value = RAW_TO_HZ };
  updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[0]);

  updateEncoderStream(stream, encoder, pinTxEn, true, data->sampleCount.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->sampleCount.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->sampleCount.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, true, data->sampleCount.bytes[0]);

  int32Value l;
  for (uint32_t i = 0; i < data->sampleCount.value; i++) {
    yeldTask();

    l.bytes[0] = data->waveform[i][0];
    l.bytes[1] = data->waveform[i][1];
    l.bytes[2] = data->waveform[i][2];
    if (l.bytes[2] & 0x80) l.bytes[3] = 0xFF;
    else l.bytes[3] = 0x00;
    f.value = l.value * RAW_TO_AMP;
    updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[3]);
    updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[2]);
    updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[1]);
    updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[0]);

    l.bytes[0] = data->waveform[i][3];
    l.bytes[1] = data->waveform[i][4];
    l.bytes[2] = data->waveform[i][5];
    if (l.bytes[2] & 0x80) l.bytes[3] = 0xFF;
    else l.bytes[3] = 0x00;
    f.value = l.value * RAW_TO_VOLT;
    updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[3]);
    updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[2]);
    updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[1]);
    updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[0]);

    l.bytes[0] = data->waveform[i][6];
    l.bytes[1] = data->waveform[i][7];
    l.bytes[2] = data->waveform[i][8];
    if (l.bytes[2] & 0x80) l.bytes[3] = 0xFF;
    else l.bytes[3] = 0x00;
    f.value = l.value * RAW_TO_VOLT;
    updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[3]);
    updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[2]);
    updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[1]);
    updateEncoderStream(stream, encoder, pinTxEn, true, f.bytes[0]);
  }

  finalizeEncoderStream(stream, encoder, pinTxEn);
  if (postTransmitAction) (*postTransmitAction)();
}

void updateEncoderStream(Stream* stream, b64_crc32_coder* encoder, int8_t pinTxEn, boolean updateCrc, uint8_t dataRaw) {
  encoder->a3[encoder->i++] = dataRaw;
  if (encoder->i >= 3) {
    a3_to_a4(encoder->a4, encoder->a3);
    for (uint8_t k = 0; k < 4; k++) {
      if (pinTxEn >= 0) while (!digitalRead(pinTxEn)) yeldTask();
      stream->write(b64_alphabet[encoder->a4[k]]);
      if (pinTxEn >= 0) stream->flush();
    }
    encoder->i = 0;
  }

  if (updateCrc) {
    encoder->crc.value ^= dataRaw;
    for (uint8_t k = 0; k < 8; k++) encoder->crc.value = encoder->crc.value & 1 ? (encoder->crc.value >> 1) ^ POLY : encoder->crc.value >> 1;
  }
}

void finalizeEncoderStream(Stream* stream, b64_crc32_coder* encoder, int8_t pinTxEn) {
  encoder->crc.value = ~encoder->crc.value;
  updateEncoderStream(stream, encoder, pinTxEn, false, encoder->crc.bytes[3]);
  updateEncoderStream(stream, encoder, pinTxEn, false, encoder->crc.bytes[2]);
  updateEncoderStream(stream, encoder, pinTxEn, false, encoder->crc.bytes[1]);
  updateEncoderStream(stream, encoder, pinTxEn, false, encoder->crc.bytes[0]);

  if (encoder->i) {
    for (uint8_t k = encoder->i; k < 3; k++) encoder->a3[k] = 0;
    a3_to_a4(encoder->a4, encoder->a3);
    for (uint8_t k = 0; k <= encoder->i; k++) {
      if (pinTxEn >= 0) while (!digitalRead(pinTxEn)) yeldTask();
      stream->write(b64_alphabet[encoder->a4[k]]);
      if (pinTxEn >= 0) stream->flush();
    }
    while (encoder->i++ < 3) {
      if (pinTxEn >= 0) while (!digitalRead(pinTxEn)) yeldTask();
      stream->write('=');
      if (pinTxEn >= 0) stream->flush();
    }
  }

  if (pinTxEn >= 0) while (!digitalRead(pinTxEn)) yeldTask();
  stream->write('\n');
  stream->flush();
}

void initCoder(b64_crc32_coder* coder) {
  coder->crc.value = 0xFFFFFFFF;
  coder->i = 0;
}

inline void a3_to_a4(uint8_t* a4, uint8_t* a3) {
  a4[0] = (a3[0] & 0xfc) >> 2;
  a4[1] = ((a3[0] & 0x03) << 4) | ((a3[1] & 0xf0) >> 4);
  a4[2] = ((a3[1] & 0x0f) << 2) | ((a3[2] & 0xc0) >> 6);
  a4[3] = (a3[2] & 0x3f);
}

inline void a4_to_a3(uint8_t* a3, uint8_t* a4) {
  a3[0] = (a4[0] << 2) | ((a4[1] & 0x30) >> 4);
  a3[1] = ((a4[1] & 0xf) << 4) | ((a4[2] & 0x3c) >> 2);
  a3[2] = ((a4[2] & 0x3) << 6) | a4[3];
}
