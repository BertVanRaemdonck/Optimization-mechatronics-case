function v = OP_ERFINV()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 92);
  end
  v = vInitialized;
end
