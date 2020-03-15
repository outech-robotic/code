from flask_wtf import FlaskForm
from wtforms import FloatField, FormField, SubmitField
from wtforms.validators import InputRequired, Optional


class PIDForm(FlaskForm):
    P = FloatField('P', default=0.0, validators=[InputRequired()])
    I = FloatField('I', default=0.0, validators=[InputRequired()])
    D = FloatField('D', default=0.0, validators=[InputRequired()])


class CapForm(FlaskForm):
    cap_speed_translation = FloatField('cap_speed_translation', default=-1, validators=[InputRequired()])
    cap_speed_rotation = FloatField('cap_speed_rotation', default=-1, validators=[InputRequired()])
    cap_speed_wheel = FloatField('cap_speed_wheel', default=-1, validators=[InputRequired()])
    cap_accel_wheel = FloatField('cap_accel_wheel', default=-1, validators=[InputRequired()])


class AllPIDForms(FlaskForm):
    Translation = FormField(PIDForm)
    Rotation = FormField(PIDForm)
    SpeedLeft = FormField(PIDForm)
    SpeedRight = FormField(PIDForm)
    CapForm = FormField(CapForm)


class OrderForm(FlaskForm):
    Speed = FloatField('Speed', validators=[Optional()])
    Position = FloatField('Position', validators=[Optional()])
    Angle = FloatField('Angle', validators=[Optional()])
