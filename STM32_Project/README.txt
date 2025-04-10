README – STM32_Project
======================
Dieses Projekt wurde größtenteils von einem anderen Studenten (Tao Zhang) erstellt, um das Pendel mittels eines PD-Reglers zu regeln. Das vorliegende Projekt stellt meine Anpassung dar, bei der anstelle des PD-Reglers ein Reinforcement-Learning-Agent implementiert wurde.


Entwicklungsumgebung:

Als Entwicklungsumgebung wurde STM32CubeIDE genutzt.
Es muss ein Projekt für den STM32F103C8T6 erstellt werden.
Anschließend müssen die Ordner (und Dateien) in das Projekt eingebunden werden.
Darauf achten, dass die Include-Pfade und die Source Locations korrekt gesetzt werden.


Integration des RL-Agenten:

Um den trainierten Agenten in das STM32-Projekt zu integrieren, müssen die C-Arrays (mit den exportierten Gewichten und Biases) in den Ordner /NN/c_arrays kopiert werden.
Danach lässt sich das Projekt kompilieren und auf dem echten System testen.


Steuerung des Motors:

Die Motorsteuerung wird in der Datei /Src/inference_control.c durchgeführt.
Falls Anpassungen an der Motorsteuerung notwendig sind, können hier die entsprechenden Änderungen vorgenommen werden.


Flashen des STM32-Boards:

Das verwendete STM32-Board verfügt nicht über einen integrierten ST-Link.
Stattdessen wird das Board über einen Bootloader geflasht.
Als empfohlenes Programm zum Flashen des STM32-Boards sollte der STM32CubeProgrammer verwendet werden.