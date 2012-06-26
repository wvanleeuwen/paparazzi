cd ../
mkdir gitsvnpprz
cd gitsvnpprz
git svn init https://svn.lr.tudelft.nl/asti/SmartUAV/Software/paparazzi3/
git svn fetch -r5885
git svn rebase
git svn dcommit
# git remote add tudelft git@github.com:tudelft/paparazzi.git


