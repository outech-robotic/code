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
                sh 'pip install -r requirements.txt'
            }
        }
        stage('Tests') {
            parallel {
                stage('unit-tests') {
                    steps {
                        sh 'mkdir reports'
                        sh 'pytest --cov=src --junitxml reports/junit.xml --cov-report xml:reports/coverage.xml src'
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
