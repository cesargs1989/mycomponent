# RCIS
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/files/makeMeCoffee'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcis simulation.xml'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'rcis'

# joystickComp
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/joystickComp/bin'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './joystickComp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/joystick.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'joystick'
sleep 1

# AprilTags
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/apriltagsComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'bin/apriltagscomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/apriltags.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'april'
sleep 1
