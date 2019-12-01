pipeline {
    agent {
        docker {
            image 'python:3.8'
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
                sh 'pipenv run pytest --cov=src --junitxml reports/junit.xml --cov-report xml:reports/coverage.xml src'
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
                sh 'pipenv run yapf -p -d -r src || (echo "!!!!!!!!!!! Please run yapf -i -r src/ on your code"; exit 1)'
            }
        }
        stage('lint') {
            steps {
                sh 'pipenv run pylint src'
            }
        }
        stage('mypy') {
            steps {
                sh 'pipenv run mypy --disallow-untyped-calls --ignore-missing-imports --disallow-incomplete-defs src'
            }
        }
    }
}
