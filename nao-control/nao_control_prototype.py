# MAIN Module that is used to configure and start the project

import mappedInput
#import naoControl



try:
    controler = mappedInput.mappedInput()
    # sim @ home
    controler.setAddress("192.168.2.16",9559)
    # sim @ MobileLab
    #controler.setAddress("192.168.8.109",9559)
    ## use setSimulatorSettings() to adjust speed and other parameters that could be different when using simulator! add changes to naoControl module
    #controler.nao.setSimulatorSettings()

    # real Nao
    #controler.setAddress("192.168.0.102",9559)

    controler.connect()
    ## setting speed has to be done in setSimulatorSettings() !!
    # following two lines needed to control simulated nao
    #controler.nao.fractionMaxSpeed = 0.3
    #controler.nao.setStiffness(1.0)


    #controler.nao.moveHeadTo([0.0,-0.3])
    #controler.nao.camera.selectCam(1)


    controler.run()

    exit(0)


except RuntimeError,e:
    print "an error occured :nao_control_prototype"
    print e
    exit(0)




