# HARDWARE

## FTD


## AP


## FORC



# Abbreviations

**AP**	Autopilot
**APPD**	Aircraft Poweron Protection Device
**FORC**	Fantastic Onboard Detection Computer
**OCC**	Onboard Cheapass Camera
**MORA**	Magic Onboard Recognition Apparatus
**POPCORN**	Onboard CHDK PTP based camera image retreival application
**SODA**	Speedy Onboard Detection Application
**CANDY**	IO application between AP and FORC
**ESC**	Electronic Speed Controller
**FTD**	Flight Termination Device
TYPE1OPS	Fully automatic one run recognition and rescue flights
TYPE2OPS	Two fased approach flights
**CEPP**	Camera External Power Provisioning
**DISO**	Direct Switch On, of e.g. a camera

# GIT

Sparse checkout

    mkdir myrepo
    cd myrepo 
    git init
    git config core.sparseCheckout true
    git remote add -f origin git://...
    echo path/to/subdir/*> .git/info/sparse-checkout
    git checkout [branchname]


