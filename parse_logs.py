import os
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt


import readlog


pp = PdfPages('logs.pdf')
    

nr = 1
dir_path = os.path.dirname(os.path.realpath(__file__))
for root, dirs, files in os.walk(dir_path):
    for file in sorted(files):
        if file.endswith(".data"):
            filename = os.path.join(root, file)
            if os.path.getsize(filename) > 100:
                res = readlog.plot_log_file(filename, nr)
                print(filename, res)
                pp.savefig(nr)
            else:
                print(filename, "EMPTY")
            plt.close('all')
            nr = nr + 1


pp.close()
