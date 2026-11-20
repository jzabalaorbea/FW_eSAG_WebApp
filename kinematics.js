window.BIKE_KINEMATICS = {
  WILD_S: {
    name: "Wild — Talla S (205x65 mm)",
    strokeMm: 65,
    eyeToEyeMaxMm: 205,
    // Incremento angular en Main Pivot (grados) para cada 1 mm de carrera (Y = 0..65 mm)
    deltaTheta: [
      0.000000, 0.438733, 0.874234, 1.306519, 1.735611, 2.161542, 2.584349, 3.004074,
      3.420763, 3.834463, 4.245225, 4.653100, 5.058141, 5.460400, 5.859931, 6.256787,
      6.651022, 7.042688, 7.431839, 7.818526, 8.202800, 8.584713, 8.964316, 9.341657,
      9.716785, 10.089749, 10.460597, 10.829375, 11.196129, 11.560905, 11.923748, 12.284702,
      12.643810, 13.001116, 13.356663, 13.710491, 14.062643, 14.413159, 14.762081, 15.109449,
      15.455302, 15.799680, 16.142622, 16.484168, 16.824356, 17.163225, 17.500813, 17.837159,
      18.172302, 18.506279, 18.839130, 19.170892, 19.501606, 19.831308, 20.160040, 20.487840,
      20.814747, 21.140804, 21.466049, 21.790525, 22.114274, 22.437338, 22.759761, 23.081587,
      23.402861, 23.723630
    ]
  }
};

window.interpolateRearShock = function interpolateRearShock(rawRelAngleDeg, modelKey = "WILD_S", direction = 1) {
  const model = window.BIKE_KINEMATICS[modelKey] || window.BIKE_KINEMATICS.WILD_S;
  const table = model.deltaTheta;
  const idle = {
    travelMm: 0,
    percent: 0,
    shockLengthMm: model.eyeToEyeMaxMm,
    deltaTheta: 0,
  };

  if (!Number.isFinite(rawRelAngleDeg) || !table || table.length < 2) {
    return idle;
  }

  const signedDeg = rawRelAngleDeg > 180 ? rawRelAngleDeg - 360 : rawRelAngleDeg;
  const deltaTheta = signedDeg * direction;

  if (deltaTheta <= 0) {
    return {
      travelMm: 0,
      percent: 0,
      shockLengthMm: model.eyeToEyeMaxMm,
      deltaTheta,
    };
  }

  const last = table[table.length - 1];
  if (deltaTheta >= last) {
    return {
      travelMm: model.strokeMm,
      percent: 100,
      shockLengthMm: model.eyeToEyeMaxMm - model.strokeMm,
      deltaTheta,
    };
  }

  let index = 0;
  while (index < table.length - 2 && table[index + 1] < deltaTheta) {
    index += 1;
  }

  const span = table[index + 1] - table[index];
  const fraction = span === 0 ? 0 : (deltaTheta - table[index]) / span;
  const travelMm = index + fraction;

  return {
    travelMm,
    percent: (travelMm / model.strokeMm) * 100,
    shockLengthMm: model.eyeToEyeMaxMm - travelMm,
    deltaTheta,
  };
};
