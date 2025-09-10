import joblib

# plain:    a list of 81569 plaintexts.  Each plaintext is a 16-character hex string
# cipher:   a list of 81569 ciphertexts. Each ciphertext is a 16-character hex string
# voltages: numpy ndarray. The first dimension is the number of traces (81569),
#           the second dimension is the number of samples in each trace (5003)
plain, cipher, voltages = joblib.load("traces.npy")
