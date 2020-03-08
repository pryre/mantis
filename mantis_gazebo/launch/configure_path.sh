# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

#!/bin/sh

MANTIS_GZ_DIR="$(rospack find mantis_gazebo)"

export GAZEBO_RESOURCE_PATH="$MANTIS_GZ_DIR/resources:$GAZEBO_RESOURCE_PATH"
export GAZEBO_MODEL_PATH="$MANTIS_GZ_DIR/models:$GAZEBO_MODEL_PATH"

