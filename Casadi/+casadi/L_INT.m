function v = L_INT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 1);
  end
  v = vInitialized;
end
