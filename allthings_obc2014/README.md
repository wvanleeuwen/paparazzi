# HARDWARE

## FTD


## AP


## FORC

# Commands

## Screen
Candy is started in a screen to be able to show the progress while running. Some basic screen screen commands:
- Shell execute **'screen -r'** will open the screen. If a list is shown multiple screens are available and name must be entered after the '-r'. The default name for Candy is 'candy'.
- When attached to a screen(when you have it open), you will be able to detach by using **ctrl+A** **ctrl+D**. This way the screen keeps running.
- When you want to close a screen simple attach to it and then press **ctrl+C**.

# Abbreviations

 - **AP**	Autopilot
 - **APPD**	Aircraft Poweron Protection Device
 - **FORC**	Fantastic Onboard Detection Computer
 - **OCC**	Onboard Cheapass Camera
 - **MORA**	Magic Onboard Recognition Apparatus
 - **POPCORN**	Onboard CHDK PTP based camera image retreival application
 - **SODA**	Speedy Onboard Detection Application
 - **CANDY**	IO application between AP and FORC
 - **ESC**	Electronic Speed Controller
 - **FTD**	Flight Termination Device
 - TYPE1OPS	Fully automatic one run recognition and rescue flights
 - TYPE2OPS	Two fased approach flights
 - **CEPP**	Camera External Power Provisioning
 - **DISO**	Direct Switch On, of e.g. a camera

# GIT

Sparse checkout

    mkdir myrepo
    cd myrepo 
    git init
    git config core.sparseCheckout true
    git remote add -f origin git://...
    echo path/to/subdir/*> .git/info/sparse-checkout
    git checkout [branchname]


