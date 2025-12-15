import type { ReactNode } from "react";
import clsx from "clsx";
import Heading from "@theme/Heading";
import styles from "./styles.module.css";

type FeatureItem = {
	title: string;
	Svg: React.ComponentType<React.ComponentProps<"svg">>;
	description: ReactNode;
};

const FeatureList: FeatureItem[] = [
	{
		title: "Specification-Driven Structure",
		Svg: require("@site/static/img/undraw_docusaurus_mountain.svg").default,
		description: (
			<>
				Each module is defined and scaffolded using Spec-Kit Plus,
				ensuring clarity, consistency, and scalability throughout the
				book.
			</>
		),
	},
	{
		title: "AI-Assisted Content Creation",
		Svg: require("@site/static/img/undraw_docusaurus_tree.svg").default,
		description: (
			<>
				Leverages Gemini CLI for generating, editing, and refining
				technical content, making the book accurate and up-to-date.
			</>
		),
	},
	{
		title: "AI-Driven Learning Experience",
		Svg: require("@site/static/img/undraw_docusaurus_react.svg").default,
		description: (
			<>
				Utilizes AI to provide interactive learning experiences,
				ensuring engagement and effective knowledge retention.
			</>
		),
	},
];

function Feature({ title, Svg, description }: FeatureItem) {
	return (
		<div className={clsx("col col--4")}>
			<div className='text--center'>
				<Svg className={styles.featureSvg} role='img' />
			</div>
			<div className='text--center padding-horiz--md'>
				<Heading as='h3'>{title}</Heading>
				<p>{description}</p>
			</div>
		</div>
	);
}

export default function HomepageFeatures(): ReactNode {
	return (
		<section className={styles.features}>
			<div className='container'>
				<div className='row'>
					{FeatureList.map((props, idx) => (
						<Feature key={idx} {...props} />
					))}
				</div>
			</div>
		</section>
	);
}
