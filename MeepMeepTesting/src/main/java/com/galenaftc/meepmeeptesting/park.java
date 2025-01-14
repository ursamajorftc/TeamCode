package com.galenaftc.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class park {
	public static void main(String[] args) {
		double pi = Math.PI;
		MeepMeep meepMeep = new MeepMeep(900);

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setColorScheme(new ColorSchemeRedDark())
				.setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 19.544)
				.build();

		myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(26, -62, Math.toRadians(90)))
				.strafeTo(new Vector2d(62, -62))
				.build());

		meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
				.setDarkMode(true)
				.setBackgroundAlpha(10f)
				.addEntity(myBot)
				.start();
	}
}