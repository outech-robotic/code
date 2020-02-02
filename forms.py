from flask_wtf import FlaskForm
from wtforms import FloatField, FormField, SubmitField
from wtforms.validators import InputRequired, Optional


class PIDForm(FlaskForm):
    P = FloatField('P', default=0.0, validators=[InputRequired()])
    I = FloatField('I', default=0.0, validators=[InputRequired()])
    D = FloatField('D', default=0.0, validators=[InputRequired()])


class AllPIDForms(FlaskForm):
    PosLeft = FormField(PIDForm)
    PosRight = FormField(PIDForm)
    SpeedLeft = FormField(PIDForm)
    SpeedRight = FormField(PIDForm)

    Submit = SubmitField()


class OrderForm(FlaskForm):
    Speed = FloatField('Speed', validators=[Optional()])
    Position = FloatField('Position', validators=[Optional()])
    Angle = FloatField('Angle', validators=[Optional()])

    Submit = SubmitField()