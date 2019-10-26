#!/bin/bash

AWS_REGION="us-east-1"
ROBOT_APP="cloudwatch_robot"
ROBOT_APP_KEY="${ROBOT_APP}/output.tar"
ROBOT_LAUNCH_FILE="deploy_rotate.launch"

robotBucketName=$(aws cloudformation describe-stacks \
    --stack-name devopstar-rpi-robot \
    --query 'Stacks[0].Outputs[?OutputKey==`RobotAppBucket`].OutputValue' \
    --region ${AWS_REGION} \
    --output text)

aws cloudformation create-stack \
    --stack-name "devopstar-rpi-robot-app-cloudwatch" \
    --template-body file://robomaker-app.yaml \
    --parameters \
        ParameterKey=Name,ParameterValue=${ROBOT_APP} \
        ParameterKey=RosBundleKeyPrefix,ParameterValue=${ROBOT_APP_KEY} \
        ParameterKey=RosBundleBucketName,ParameterValue=${robotBucketName} \
    --region ${AWS_REGION}

aws cloudformation wait stack-create-complete \
    --region ${AWS_REGION} \
    --stack-name "devopstar-rpi-robot-app-cloudwatch"

fleetArn=$(aws cloudformation describe-stacks \
    --stack-name devopstar-rpi-robot \
    --query 'Stacks[0].Outputs[?OutputKey==`RoboMakerFleet`].OutputValue' \
    --region ${AWS_REGION} \
    --output text)
applicationArn=$(aws cloudformation describe-stacks \
    --stack-name devopstar-rpi-robot-app-cloudwatch \
    --query 'Stacks[0].Outputs[?OutputKey==`RobotApplication`].OutputValue' \
    --region ${AWS_REGION} \
    --output text)
applicationVersion=$(aws cloudformation describe-stacks \
    --stack-name devopstar-rpi-robot-app-cloudwatch \
    --query 'Stacks[0].Outputs[?OutputKey==`RobotApplicationVersion`].OutputValue' \
    --region ${AWS_REGION} \
    --output text)

# Generate deployment configuration
cat <<EOT > deployment.json         
{
    "fleet": "${fleetArn}",
    "deploymentApplicationConfigs": [
        {
            "application": "${applicationArn}",
            "applicationVersion": "${applicationVersion: -1}",
            "launchConfig": {
                "packageName": "${ROBOT_APP}",
                "launchFile": "${ROBOT_LAUNCH_FILE}"
            }
        }
    ]
}
EOT

# Deploy configuration
aws robomaker create-deployment-job \
    --region ${AWS_REGION} \
    --cli-input-json file://deployment.json
