#!/bin/bash

AWS_REGION="us-east-1"

greengrassGroupId=$(aws cloudformation describe-stacks \
    --stack-name devopstar-rpi-gg-core \
    --query 'Stacks[0].Outputs[?OutputKey==`GreengrassGroupId`].OutputValue' \
    --region ${AWS_REGION} \
    --output text)

aws cloudformation create-stack \
    --stack-name "devopstar-rpi-robot" \
    --template-body file://robomaker.yaml \
    --parameters \
        ParameterKey=GreengrassGroupId,ParameterValue=${greengrassGroupId} \
    --region ${AWS_REGION}

aws cloudformation wait stack-create-complete \
    --region ${AWS_REGION} \
    --stack-name "devopstar-rpi-robot"

robotBucketName=$(aws cloudformation describe-stacks \
    --stack-name devopstar-rpi-robot \
    --query 'Stacks[0].Outputs[?OutputKey==`RobotAppBucket`].OutputValue' \
    --region ${AWS_REGION} \
    --output text)

echo "Put robot deployment files in s3://${robotBucketName}"
