# include <Siv3D.hpp>

struct Physics
{
	double mass;
	Vec3 pos;
	Vec3 vel;
	Vec3 force;
	OrientedBox obj;

	Physics() : mass(0), vel({ 0,0,0 }), force({ 0,0,0 }), obj(OrientedBox{ {0,0,0},{1,1,1} })
	{
		pos = obj.center;
	}

	/// @param mass 質量
	/// @param vel 速度
	/// @param force 力
	/// @param obj OrientedBox: {center, size}
	Physics(double mass, Vec3 vel, Vec3 force, OrientedBox obj)
		: mass(mass), vel(vel), force(force), obj(obj)
	{
		pos = obj.center;
	}

	virtual void draw() const
	{
		//Print << U"Physics: virtual draw";
		obj.draw();
	}

	virtual void draw(const Texture& tex) const
	{
		obj.draw(tex);
	}

	virtual void draw(const ColorF& color) const
	{
		obj.draw(color);
	}

	virtual void update()
	{
		double dt = Scene::DeltaTime();
		vel += (force / mass) * dt;
		pos += vel * dt;
		obj.setPos(pos);
	}

	void stop()
	{
		vel = Vec3{ 0, 0, 0 };
		force = Vec3{ 0, 0, 0 };
	}

	static Optional<Vec3> getNormal(const Vec3& point, const Physics physics)
	{
		// pointがphysicsのどの面にあるかを判定
		// {0,0,0}を中心としgetCorners()で得られる頂点のうち、インデックスが
		// {0,1,2,3}が -Z	{4,5,6,7}が +Z
		// {0,4,2,6}が -X	{1,5,3,7}が +X
		// {2,3,6,7}が -Y	{0,1,4,5}が +Yの方向
		// 角に当たったときの処理は不定

		Optional<Vec3> ret;
		int indexs[6][4] = {{0,1,2,3}, {4,5,6,7}, {0,4,2,6}, {1,5,3,7}, {2,3,6,7}, {0,1,4,5}};
		Array<Vec3> normals = { {0,0,-1}, {0,0,1}, {-1,0,0}, {1,0,0}, {0,-1,0}, {0,1,0} };
		Quaternion rot = physics.obj.orientation;
		for (int i = 0; i < normals.size(); i++) normals[i] = normals[i] * rot;

		std::array<Vec3, 8> corners = physics.obj.getCorners();
		double eps = 1e-5;
		for (int i = 0; i < 6; i++)
		{
			Vec3 A = corners[indexs[i][0]];
			Vec3 B = corners[indexs[i][1]];
			Vec3 C = corners[indexs[i][2]];

			Vec3 AB = B - A;
			Vec3 AC = C - A;
			Vec3 AP = point - A;

			// 行列式が0 == 四面体の体積は0 == 点は平面上にある
			Mat3x3 mat;
			mat._11_12_13 = AB;
			mat._21_22_23 = AC;
			mat._31_32_33 = AP;

			Print << U"mat: " << mat.determinant();
			if (Abs(mat.determinant()) < eps) {
				ret = normals[i];
				Print << U"normal: " << normals[i];
			}
		}

		return ret;

	}

	void reflect(Physics Wall, double e)
	{
		// Wallは不動とする
		Vec3 v = this->vel;
		Ray ray{this->pos, v.normalized()};
		Optional<Float3> intersect = ray.intersectsAt(Wall.obj);
		if (!intersect) return;

		Optional<Vec3> n = getNormal(*intersect, Wall);
		if (!n) return;

		Vec3 N = *n;
		this->vel = v - (1+e) * Dot(v, N) * N;
		Print << U"reflect1";
	}

	static void collision(Physics &p1, Physics &p2, double e)
	{
		Vec3 v1 = p1.vel;
		Vec3 v2 = p2.vel;
		double m1 = p1.mass;
		double m2 = p2.mass;

		Ray ray{ p1.pos, v1.normalized() };
		Optional<Float3> intersect = ray.intersectsAt(p2.obj);
		if (!intersect) {
			Print << U"no intersect";
			return;
		}

		Optional<Vec3> n = getNormal(*intersect, p2);
		if (!n) {
			Print << U"no normal";
			return;
		}

		// 外力を一旦無視して運動量保存則から速度を求める
		Vec3 N = *n;
		Vec3 v1_after = (m1*v1 + m2*v2 - m2*e*(v1-v2)) / (m1+m2);
		Vec3 v2_after = (m1*v1 + m2*v2 + m1*e*(v1-v2)) / (m1+m2);
		p1.vel = v1_after * N;
		p2.vel = v2_after;

	}
	
};

struct Test : Physics
{
	Test () : Physics() {}
	Test(double mass, Vec3 vel, Vec3 force, OrientedBox obj)
		: Physics(mass, vel, force, obj) {}

	void draw() const override
	{

	}

	void update() override
	{
		Physics::update();
	}
};

void Main()
{
	//Window::Resize(1280, 720);
	const ColorF backgroundColor = ColorF{ 0.4, 0.6, 0.8 }.removeSRGBCurve();
	const Texture uvChecker{ U"example/texture/uv.png", TextureDesc::MippedSRGB };
	const MSRenderTexture renderTexture{ Scene::Size(), TextureFormat::R8G8B8A8_Unorm_SRGB, HasDepth::Yes };
	DebugCamera3D camera{ renderTexture.size(), 120_deg, Vec3{ 0, 2, -5 } };

	Physics p1{ 1, {0,0,0}, {4,1,0}, OrientedBox{ {-4,-1,0}, {1,1,1} } };
	Physics p2{ 1, {0,0,0}, {-4,-1,0}, OrientedBox{ {4,1,0}, {1,1,1}}};

	while (System::Update())
	{
		ClearPrint();
		camera.update(2.0);
		Graphics3D::SetCameraTransform(camera);

		// 3D 描画
		{
			const ScopedRenderTarget3D target{ renderTexture.clear(backgroundColor) };
			{
				const ScopedRenderStates3D blend{ BlendState::OpaqueAlphaToCoverage };

				p1.update();
				p2.update();
				p1.draw(Palette::Red);
				p2.draw();

				if(p1.obj.intersects(p2.obj)) Physics::collision(p1, p2, 0.5);

				Print << U"IJKL: rotate p2, WSAD: move camera, R: retry";
				if (KeyJ.down()) {
					p1.pos = { 0,4,0 };
					p1.vel = { 0,0,0 };
					p2.obj.orientation *= Quaternion::RotateZ(10_deg);
				}
				if (KeyL.down()) {
					p1.pos = { 0,4,0 };
					p1.vel = { 0,0,0 };
					p2.obj.orientation *= Quaternion::RotateZ(-10_deg);
				}
				if (KeyI.down()) {
					p1.pos = { 0,4,0 };
					p1.vel = { 0,0,0 };
					p2.obj.orientation *= Quaternion::RotateX(10_deg);
				}
				if (KeyK.down()) {
					p1.pos = { 0,4,0 };
					p1.vel = { 0,0,0 };
					p2.obj.orientation *= Quaternion::RotateX(-10_deg);
				}
				if (KeyR.down()) {
					p1.pos = { 0,4,0 };
					p1.vel = { 0,0,0 };
					p2.pos = { 0,0,0 };
					p2.vel = { 0,0,0 };
				}
			}
		}

		// 3D シーンを 2D シーンに描画
		{
			Graphics3D::Flush();
			renderTexture.resolve();
			Shader::LinearToScreen(renderTexture);
		}

	}
}

