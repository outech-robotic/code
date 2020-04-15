pipeline {
    agent {
        docker {
            image 'python:3.7'
        }
    }
    stages {
        stage('build') {
            steps {
                sh 'python --version'
                sh 'pip install pipenv'
                sh 'pipenv install'
            }
        }
        stage('unit-tests') {
            steps {
                sh 'mkdir reports'
                sh 'pipenv run pytest --cov=highlevel --junitxml reports/junit.xml --cov-report xml:reports/coverage.xml highlevel'
            }
            post {
                always {
                    junit 'reports/*.xml'
                    step([$class: 'CoberturaPublisher',
                                   autoUpdateHealth: false,
                                   autoUpdateStability: false,
                                   coberturaReportFile: 'reports/coverage.xml',
                                   failNoReports: false,
                                   failUnhealthy: false,
                                   failUnstable: false,
                                   maxNumberOfBuilds: 10,
                                   onlyStable: false,
                                   sourceEncoding: 'ASCII',
                                   zoomCoverageChart: false])
                }
            }
        }
        stage('ensure code is yapf formatted') {
            steps {
                sh 'pipenv run yapf -p -d -r highlevel || (echo "!!!!!!!!!!! Please run yapf -i -r highlevel/ on your code"; exit 1)'
            }
        }
        stage('lint') {
            steps {
                sh 'pipenv run pylint highlevel'
            }
        }
        stage('mypy') {
            steps {
                sh 'pipenv run mypy --disallow-untyped-calls --ignore-missing-imports --disallow-incomplete-defs highlevel'
            }
        }
    }
}
