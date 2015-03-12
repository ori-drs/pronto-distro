#!/bin/bash
# Launch designer and envoke environment

source /home/mfallon/drc/software/config/drc_environment.sh 
drake-designer --director_config /home/mfallon/drc/software/models/thor_mang_description/director_config.json -c /home/mfallon/otherprojects/pronto-distro/config/thor_mang.cfg
