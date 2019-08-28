pipeline {
    agent { docker { image 'python:3.7' } }
    stages {
        stage('build') {
            steps {
                sh 'python --version'
                sh 'pip install -r requirements.txt'
            }
        }
        stage('Tests') {
            parallel {
                stage('unit-tests') {
                    steps {
                        sh 'pytest src'
                    }
                }
                stage('lint') {
                    steps {
                        sh 'pylint src'
                    }
                }
                stage('mypy') {
                    steps {
                        sh 'mypy --ignore-missing-imports src'
                    }
                }
            }
        }
    }
}
