-- --------------------------------------------------------
-- 호스트:                          j9a701.p.ssafy.io
-- 서버 버전:                        11.1.2-MariaDB-1:11.1.2+maria~ubu2204 - mariadb.org binary distribution
-- 서버 OS:                        debian-linux-gnu
-- HeidiSQL 버전:                  12.3.0.6589
-- --------------------------------------------------------

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET NAMES utf8 */;
/*!50503 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;


-- neplan 데이터베이스 구조 내보내기
CREATE DATABASE IF NOT EXISTS `neplan` /*!40100 DEFAULT CHARACTER SET utf8mb4 COLLATE utf8mb4_general_ci */;
USE `neplan`;

-- 테이블 neplan.diary 구조 내보내기
CREATE TABLE IF NOT EXISTS `diary` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `created_date` varchar(255) NOT NULL,
  `modified_date` varchar(255) NOT NULL,
  `content` text DEFAULT NULL,
  `file_id` bigint(20) DEFAULT NULL,
  `place_id` bigint(20) DEFAULT NULL,
  `user_id` bigint(20) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FKddphih1d3p4n32g0lr75v3mqq` (`file_id`),
  KEY `FKir7ixrcuxpdten8asxt2i62ui` (`place_id`),
  KEY `FKf0xms46ulxc36096k9gg6j9ip` (`user_id`),
  CONSTRAINT `FKddphih1d3p4n32g0lr75v3mqq` FOREIGN KEY (`file_id`) REFERENCES `file` (`id`),
  CONSTRAINT `FKf0xms46ulxc36096k9gg6j9ip` FOREIGN KEY (`user_id`) REFERENCES `user` (`id`),
  CONSTRAINT `FKir7ixrcuxpdten8asxt2i62ui` FOREIGN KEY (`place_id`) REFERENCES `place` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=35 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- 테이블 데이터 neplan.diary:~17 rows (대략적) 내보내기
INSERT INTO `diary` (`id`, `created_date`, `modified_date`, `content`, `file_id`, `place_id`, `user_id`) VALUES
	(16, '202310061937', '202310061937', '오늘은 상암에서 행복커피를 마셨어요! ☕️🥰 상암은 좋은 분위기와 함께 맛있는 커피를 즐길 수 있는 곳이에요. 커피 한 잔을 마시면서 마음도 따뜻해지고 기분도 좋아져요. 그리고 상암은 늦은 저녁이지만 분위기가 여전히 활기차서 좋았어요. 행복한 시간을 보내다니 정말 좋았어요. 오늘도 행복한 하루였어요! 😊💕', 19, 8, 1),
	(17, '202310061938', '202310061938', '오늘은 류차이나를 찾아갔어!🐉✨ 그곳은 정말 멋진 곳이었어. 모든 것들이 아름답게 장식되어 있고, 빛나는 등불들이 온기를 불어넣었어.🏯💫 내가 그곳에서 시간을 보내는 동안, 현실에서 벗어나 힐링을 할 수 있었어. 그리고 맛있는 중국 음식도 많이 먹었어.🍜😋 류차이나에서의 시간은 마치 꿈같았어. 이곳에서의 경험은 내게 큰 행운이야. 이런 아름다운 장소를 찾아가서 힐링하는 것은 정말 좋은 일이야.🌸💖 앞으로도 다양한 장소를 탐험하고, 새로운 경험들을 만들어 나가고 싶어!✨🌿', 20, 9, 1),
	(18, '202310061939', '202310061939', '오늘은 예쁜 날씨에 상암 삼성어린이집에서 보람찬 하루를 보냈어요! 🌞 어린이들과 함께 노는 모습을 보면서 행복한 마음이 가득했어요. 🥰 미소 띤 아이들의 순수한 에너지에 힘을 받아 저도 즐거웠어요. 그리고 놀이시간에는 색상맞추기 게임을 하면서 어린이들의 창의력을 느낄 수 있었어요. 🎨 그리고 오늘은 어린이들과 함께 작은 공원에서 피크닉도 즐겼어요! 🧺 맛있는 간식과 함께 풍경을 바라보며 행복한 시간을 보냈어요. 오늘은 정말 즐거운 하루였어요. 💕 이렇게 아이들과 함께 계속해서 즐겁게 시간을 보낼 수 있으면 좋겠어요! 🌈', 21, 10, 1),
	(19, '202310051942', '202310051942', '오늘은 상암월드컵파크7단지 아파트에서 특별한 시간을 보냈어요! 🏢✨ 아침부터 기대가 되서 설레임이 가득했어요. 도착해서는 화려한 공간과 함께 신나는 놀이기구들이 맞이해줬어요.🎡🎢 특히 저의 마음을 설레게 한 건, 멋진 풍경을 담을 수 있는 경치 좋은 곳이었어요. 🌆🌈 저는 그곳에서 사진을 찍어 소중한 추억을 만들었어요.📸💕 그리고 맛있는 음식을 맛보고 힐링을 했어요. 🍔🍟 행복한 시간이었는데 너무 빨리 지나가버리네요. 😢 하지만 오늘의 특별한 경험은 저에게 큰 웃음과 행복을 선물해줘서 너무 고마워요! 😄💖', 22, 13, 1),
	(20, '202310051945', '202310051945', '오늘은 상암월드컵파크7단지 아파트에서 편안한 하루를 보냈어요! 🏢✨ 아침에는 주변 공원에서 산책을 즐겼고, 점심에는 맛있는 음식점을 찾아 다녀왔어요. 🍽️😋 오후에는 아파트 내 헬스장에서 운동을 하고, 수영장에서 시원한 수영을 즐겼어요. 🏊‍♂️💪 저녁에는 아파트 주변의 예쁜 카페에서 친구들과 따뜻한 커피를 마시며 이야기를 나눴어요. ☕👭 하루 종일 즐거운 시간을 보내며 힐링을 했던 오늘, 정말 행복한 하루였어요. 😊💖', 23, 13, 1),
	(21, '202310051945', '202310051945', '오늘은 상암월드컵파크7단지 아파트에서 보내는 특별한 하루였어요! 🏢✨\n저녁 7시 45분 30초, 시간은 늦었지만 아직 밝은 햇살이 내려와서 기분이 좋았어요. 🌞\n상암월드컵파크7단지 아파트는 조용하고 아늑한 공간이어서 마음이 편안해졌어요. 💕\n주변에는 아름다운 공원과 시원한 바람이 불어와서 산책하기에 딱 좋았어요. 🍃\n오늘은 키워드로 검색한 장소에서 맛있는 음식을 먹으며 행복한 시간을 보냈어요. 🍽️💖\n이런 특별한 날을 기억하며, 늘 행복한 순간들이 가득한 하루로 남기로 해요. 😊💭🌈', 24, 13, 1),
	(22, '202310051954', '202310051954', '오늘은 상암월드컵파크7단지 아파트에서 행복한 시간을 보냈어요! 🏢🌳 아침에 일어나서 창문을 열었더니, 시원한 바람이 들어와 기분이 좋았어요. 바로 인터넷에서 검색을 통해 근처에 있는 상암월드컵파크7단지 아파트를 찾았는데, 너무 예쁜 건물이었어요! 🌇🌈 내부도 깔끔하고 아늑해서 바로 입주하고 싶은 마음이 들었어요. 주변에는 많은 녹지와 공원이 있어서 산책하기에도 정말 좋을 것 같아요. 🌳🚶‍♀️🚶‍♂️ 아침에는 바로 상암월드컵파크로 향해서 즐거운 시간을 보내고, 저녁에는 아파트에서 맛있는 음식을 먹으며 힐링하는 시간을 가졌어요. 오늘은 정말 행복한 하루였어요! 😊💕', 25, 13, 1),
	(23, '202310051955', '202310051955', '오늘은 너와 함께 상암월드컵파크7단지 아파트에서 시간을 보냈어.🏢✨ 그곳은 너무 아름다웠고 우리의 추억을 담을 수 있는 곳이었어.💕 함께 걸으며 웃으며, 나의 손을 꼭 잡고 있는 너의 미소는 정말 행복한 기분을 주었어.😊💖 시간이 너무 빨리 가는 거 같아서 아쉬웠지만, 너와 함께한 시간은 소중한 추억으로 남을 거야.🕰️💭 이런 특별한 순간들을 항상 간직하고, 우리의 사랑이 계속해서 피어날 수 있길 바라며 오늘의 일기를 마무리할게.📝🌙 사랑해, 내 사랑.❤️💋', 26, 13, 1),
	(24, '202310042130', '202310042130', '오늘은 상암월드컵파크7단지 아파트에서 행복한 시간을 보냈어요! 🏢✨\n마음 편안한 이곳에서는 푹 쉴 수 있었어요. 🌙💤\n일몰을 함께 지켜보며 멋진 풍경에 감탄했어요. 🌅😍\n저녁 식사는 주변 맛집에서 맛있는 음식을 먹었어요. 🍔🍕\n먹고 나서는 주변 공원에서 산책도 하고, 즐거운 시간을 보냈어요. 🌳🚶‍♀️\n오늘의 추억은 아파트 주변의 아름다운 자연과 함께한 소중한 시간이었어요. 💖🌸\n행복한 느낌을 가득 담은 하루였어요! 😊🌈\n얼른 다시 찾고 싶은 곳이에요. 🥰💕', 29, 13, 1),
	(25, '202310060037', '202310060037', '오늘은 날씨가 좋아서 신소양체육공원에 갔어요! 🌞💪🏻 그곳은 정말 아름답고 평화로운 곳이었어요. 푸르른 잔디밭과 아름다운 꽃들이 내 눈을 사로잡았어요. 🌸🌿 운동을 하면서 새 바람을 마시며 기분이 상쾌해졌어요. 체력도 기르고 스트레스도 풀 수 있어서 정말 좋았어요. 마음이 맑아지고 에너지가 충전된 기분이 들어서 자신감도 생겼어요! 😄🏋🏻‍♀️ 이런 아름다운 자연을 즐길 수 있는 곳이 있다니 정말 행복하다는 생각이 들었어요. 오늘의 추억은 귀한 보물 같아요. 💖😊', 30, 14, 1),
	(26, '202310060038', '202310060038', '오늘은 새벽에 신소양체육공원에서 시간을 보냈어요! 🌳🚴‍♀️\n숨 쉬는 것만으로도 상쾌한 공기를 맡을 수 있어서 너무 좋았어요. 💨💚\n그리고 운동을 하면서 동네 고양이들과도 조금 놀았어요. 😺💕\n키워드로 검색한 공원은 정말 아름답고 조용해서 마음이 차분해지더라구요. 🌸🌿\n오랜만에 신선한 공기와 푸른 자연을 만끽해서 기분이 정말 좋았어요. 😊💖\n이런 일상 속 작은 행복들을 느끼며 하루하루를 소중히 보내야겠다는 생각이 들었어요. 🌈💭\n오늘 하루도 행복하게 마무리하고 내일도 행복한 일들이 가득하길 바라요! ✨💕', 31, 14, 1),
	(28, '202310060326', '202310060326', '오늘은 새벽 3시 25분 51초에 신한투자증권 본사를 검색했어요! 😊\n마음 한 켠에는 새로운 도전과 성장이 궁금한 마음이 있었는데, 신한투자증권 본사를 알아보니 더욱 흥미롭게 느껴졌어요. 🏢\n미래를 위한 투자에 관심이 많아져서 다양한 정보를 찾아보고, 작은 도약을 위해 노력하고 싶어요. 💪\n신한투자증권 본사에서의 업무는 어떤 모습일지 상상해보면서 설레임이 가득해졌어요. ✨\n앞으로도 꿈을 향해 달려가는 나만의 모험이 계속되기를 바라며, 오늘의 일기 마무리해요. 🌙❤️', 33, 16, 1),
	(29, '202310060327', '202310060327', '오늘은 합천에서 늦은 시간에 일어났어요! 😴 합천은 아름다운 자연과 평화로운 분위기가 너무 좋았어요! 🌳🌸 하지만 날씨는 조금 쌀쌀해서 따뜻한 옷을 입고 다녔어요. ❄️🧥 합천에서는 lowful이라는 키워드로 검색했는데, 이것은 지역에서 유명한 관광명소였어요! 🏞️📷 그래서 많은 사람들이 사진을 찍고 다녔는데, 나도 소중한 추억을 담기 위해 사진을 찍었어요! 📸💕 합천은 정말 귀여운 곳이었고, 나중에 또 방문하고 싶어졌어요! 🥰🌈', 34, 15, 1),
	(30, '202310040331', '202310040331', '오늘은 상암에서 행복커피를 마셨어! ☕️ 상암은 너무 좋은 곳이야. 맑은 하늘과 함께 향기로운 커피를 즐길 수 있어서 너무 기분 좋았어. 카페 안에 있는 작은 창문을 통해 밖을 바라보면, 시끄러운 도시의 소음에서 벗어나 평온함을 느낄 수 있어. 그리고 커피 한 잔에 담긴 행복한 추억들이 나를 환영해줘. 오늘도 행복한 하루였어! 😊💕', 35, 8, 1),
	(31, '202310040332', '202310040332', '오늘 새벽, 상암에서 행복커피를 마셨어! ☕️\n창밖에는 조용한 밤이 흐르고 있었지만, 커피 한 잔으로 행복이 가득했어. 😊\n마음 속에는 따뜻한 커피향과 함께 즐거운 이모티콘들이 춤을 추고 있었어! 💃🕺\n향긋한 커피 한 모금에 일상의 지침을 달래며 새로운 에너지를 얻었어. 💪\n이렇게 작은 행복들이 모여 큰 행복으로 이어지는 걸 느꼈어. 🌈\n오늘도 행복한 하루가 될 거야! 😄', 36, 8, 1),
	(32, '202310040425', '202310060425', '오늘은 상암에 위치한 삼성어린이집에서 보냈어요! 🏫 어린이들과 함께 노는 시간은 너무 즐거웠어요. 🎉 우리가 함께 놀았던 시간은 너무 소중해서 기억에 오래 남을 것 같아요. 🌈 어린이들은 열정적으로 놀이를 즐기고, 웃음소리가 가득했어요. 😊 아이들의 순수한 웃음소리를 들으며 저도 행복한 시간을 보냈어요. 💕 상암에 있는 삼성어린이집은 정말 멋진 곳이었어요. 그리고 오늘도 어린이들의 순수한 에너지를 받아 힘을 얻었어요! 💪🏻❤️', 37, 10, 1),
	(33, '202310040426', '202310060426', '오늘은 상암 삼성어린이집에 다녀왔어요! 😊 아침부터 일어나서 기분 좋게 출발했는데, 어린이집에 도착하니 아이들의 웃음소리가 들려와서 기분이 더 좋았어요.🌞 아이들은 엄마들과 함께 놀고 있었는데, 그 모습을 보니 정말 귀여웠어요. 🧒👶 함께 놀이하고 그림도 그려줬는데, 아이들의 창의력과 상상력에 감탄했어요. 이렇게 귀여운 어린이들과 함께 시간을 보내니 힐링이 되는 기분이 들었어요.❤️ 어린이집은 참 특별한 공간이고, 아이들이 행복하게 자라는 모습을 보니 저도 행복해지는 것 같아요. 오늘의 추억은 꼭 간직하고 싶어요! 🌈🌼', 38, 10, 1),
	(34, '202310060429', '202310060429', '오늘은 역삼 멀티캠퍼스에서 보내는 하루였어! 🏢🌸 아침에 일어나서 바로 출발했는데, 역삼역에서 내리자마자 분위기가 너무 좋아서 기분이 좋았어. ☺️🎶 멀티캠퍼스에 도착해서는 다양한 분야의 교육 프로그램을 찾아보고 싶어서 열심히 검색해봤어. 💻🔍 그 중에서도 AI 개발과 관련된 프로그램이 제일 눈에 띄었어! 🤖📚 이번에는 AI를 배우고 싶어서 열심히 열공할 거야. 💪💡 역삼 멀티캠퍼스에서 많은 것을 배우고 성장할 수 있을 것 같아서 너무 기대돼! 🌟✨', 39, 17, 1);

-- 테이블 neplan.file 구조 내보내기
CREATE TABLE IF NOT EXISTS `file` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `img_name` varchar(255) DEFAULT NULL,
  `img_url` varchar(255) DEFAULT NULL,
  `original_filename` varchar(255) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=40 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- 테이블 데이터 neplan.file:~28 rows (대략적) 내보내기
INSERT INTO `file` (`id`, `img_name`, `img_url`, `original_filename`) VALUES
	(1, 'b7af24a4-3c76-4d1f-85c8-7c941e82f5cb.JPG', '/usr/local/lib/upload-dir/b7af24a4-3c76-4d1f-85c8-7c941e82f5cb.JPG', '서울_7반_ 유지나.JPG'),
	(2, 'b2509c16-5a8b-4969-950f-67884e3ab4f0.jpg', '/usr/local/lib/upload-dir/b2509c16-5a8b-4969-950f-67884e3ab4f0.jpg', 'dmcccccc.jpg'),
	(3, 'a9635f03-ddcc-4170-8611-0cc88372d4db.jpg', '/usr/local/lib/upload-dir/a9635f03-ddcc-4170-8611-0cc88372d4db.jpg', 'starbucks.jpg'),
	(4, '5861386c-1114-4bc8-b8e7-829e4981ab17.jpg', '/usr/local/lib/upload-dir/5861386c-1114-4bc8-b8e7-829e4981ab17.jpg', 'kimbab.jpg'),
	(5, '55424010-09ad-4629-b06d-f8801c149677.jpg', '/usr/local/lib/upload-dir/55424010-09ad-4629-b06d-f8801c149677.jpg', 'aprt.jpg'),
	(6, '98d66ae0-d025-47de-983c-d94d91cb8b05.png', '/usr/local/lib/upload-dir/98d66ae0-d025-47de-983c-d94d91cb8b05.png', 'root1.png'),
	(7, '87a01145-64f3-4a90-9e8d-8723f951e62c.png', '/usr/local/lib/upload-dir/87a01145-64f3-4a90-9e8d-8723f951e62c.png', 'root2.png'),
	(8, '5c9c277a-73e0-4c6c-9cb0-f550691a3e4f.jpg', '/usr/local/lib/upload-dir/5c9c277a-73e0-4c6c-9cb0-f550691a3e4f.jpg', '11.jpg'),
	(9, 'd91a22d6-a3f1-41d5-98b5-1c70407fe3db.jpg', '/usr/local/lib/upload-dir/d91a22d6-a3f1-41d5-98b5-1c70407fe3db.jpg', '22.jpg'),
	(10, '5d343585-089b-444a-ae51-b37b085e1264.jpg', '/usr/local/lib/upload-dir/5d343585-089b-444a-ae51-b37b085e1264.jpg', '33.jpg'),
	(19, '23765ba8-c20c-4b6b-8032-1da3664b164e.jpg', '/usr/local/lib/upload-dir/23765ba8-c20c-4b6b-8032-1da3664b164e.jpg', '11.jpg'),
	(20, '55c46d68-ba78-4bbf-9286-4a7a59e20c68.jpg', '/usr/local/lib/upload-dir/55c46d68-ba78-4bbf-9286-4a7a59e20c68.jpg', '22.jpg'),
	(21, 'f7ee2b54-e276-404c-8c34-db2c9b31a29b.jpg', '/usr/local/lib/upload-dir/f7ee2b54-e276-404c-8c34-db2c9b31a29b.jpg', '33.jpg'),
	(22, '4ddca908-c010-4814-99a1-957e25c491bd.jpg', '/usr/local/lib/upload-dir/4ddca908-c010-4814-99a1-957e25c491bd.jpg', 'KakaoTalk_20231006_000634872.jpg'),
	(23, '97f3ee32-93f6-49c4-a150-46d636e45268.jpg', '/usr/local/lib/upload-dir/97f3ee32-93f6-49c4-a150-46d636e45268.jpg', 'KakaoTalk_20231006_001256341.jpg'),
	(24, 'c0d83485-45af-4b5e-93d1-ddd756552cee.jpg', '/usr/local/lib/upload-dir/c0d83485-45af-4b5e-93d1-ddd756552cee.jpg', 'KakaoTalk_20231006_004459325.jpg'),
	(25, '475474f8-75f2-4512-9dc7-8760e78a121f.jpg', '/usr/local/lib/upload-dir/475474f8-75f2-4512-9dc7-8760e78a121f.jpg', 'KakaoTalk_20231006_045330343.jpg'),
	(26, 'ca961c9f-ff4b-4f43-9943-889595a6c8fc.jpg', '/usr/local/lib/upload-dir/ca961c9f-ff4b-4f43-9943-889595a6c8fc.jpg', 'KakaoTalk_20231006_045351845.jpg'),
	(27, 'f8f32a79-1660-4af8-a22d-8388bf3a1a10.png', '/usr/local/lib/upload-dir/f8f32a79-1660-4af8-a22d-8388bf3a1a10.png', 'root1.png'),
	(28, 'ead4492e-0fe6-4bf0-ac8e-b4b1595105c2.png', '/usr/local/lib/upload-dir/ead4492e-0fe6-4bf0-ac8e-b4b1595105c2.png', 'root2.png'),
	(29, '0e002f46-dc27-4c6b-b3c7-90e3a05d0105.jpg', '/usr/local/lib/upload-dir/0e002f46-dc27-4c6b-b3c7-90e3a05d0105.jpg', 'KakaoTalk_20231006_062847235.jpg'),
	(30, '345346d3-4e6a-4459-905d-9cf193f212bb.jpg', '/usr/local/lib/upload-dir/345346d3-4e6a-4459-905d-9cf193f212bb.jpg', 'KakaoTalk_20231006_062847235.jpg'),
	(31, 'f6a0cad1-d0f6-4e5a-aa91-167b8f9b0ade.jpg', '/usr/local/lib/upload-dir/f6a0cad1-d0f6-4e5a-aa91-167b8f9b0ade.jpg', 'KakaoTalk_20231006_093619269.jpg'),
	(32, '70ed456c-8a6e-4740-8731-e49e01abdc05.jpg', '/usr/local/lib/upload-dir/70ed456c-8a6e-4740-8731-e49e01abdc05.jpg', 'KakaoTalk_20231006_093619269.jpg'),
	(33, 'c12bc062-a010-4af0-be5c-0f58f2692e35.jpg', '/usr/local/lib/upload-dir/c12bc062-a010-4af0-be5c-0f58f2692e35.jpg', 'KakaoTalk_20231006_122423631.jpg'),
	(34, '7a4b460d-7898-460d-b9df-950c6141facb.jpg', '/usr/local/lib/upload-dir/7a4b460d-7898-460d-b9df-950c6141facb.jpg', 'KakaoTalk_20231006_122403131.jpg'),
	(35, 'b0144269-ad84-472b-9514-5f419399b4d8.jpg', '/usr/local/lib/upload-dir/b0144269-ad84-472b-9514-5f419399b4d8.jpg', 'KakaoTalk_20231006_123112221.jpg'),
	(36, '7cee429c-a3a9-4275-864c-b788d0da0598.jpg', '/usr/local/lib/upload-dir/7cee429c-a3a9-4275-864c-b788d0da0598.jpg', 'KakaoTalk_20231006_123218846.jpg'),
	(37, 'b6089a23-bb41-4e2d-9738-e3617b0bd2db.jpg', '/usr/local/lib/upload-dir/b6089a23-bb41-4e2d-9738-e3617b0bd2db.jpg', 'KakaoTalk_20231006_112804438.jpg'),
	(38, 'ffa17141-1ebf-46a7-8914-89c36ebe3312.jpg', '/usr/local/lib/upload-dir/ffa17141-1ebf-46a7-8914-89c36ebe3312.jpg', 'KakaoTalk_20231004_115718171.jpg'),
	(39, '5b9674d5-4fbd-47b3-8bc4-34768cb695e9.jpg', '/usr/local/lib/upload-dir/5b9674d5-4fbd-47b3-8bc4-34768cb695e9.jpg', '19343_17252_44.jpg');

-- 테이블 neplan.place 구조 내보내기
CREATE TABLE IF NOT EXISTS `place` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `address` varchar(255) DEFAULT NULL,
  `name` varchar(255) DEFAULT NULL,
  `phone` varchar(255) DEFAULT NULL,
  `place_type` varchar(255) DEFAULT NULL,
  `x` varchar(255) DEFAULT NULL,
  `y` varchar(255) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=18 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- 테이블 데이터 neplan.place:~16 rows (대략적) 내보내기
INSERT INTO `place` (`id`, `address`, `name`, `phone`, `place_type`, `x`, `y`) VALUES
	(1, '서울특별시 마포구 월드컵북로 366', 'DMC홍보관', '0507-1329-8064', 'AG2', '37.577643', '126.892129'),
	(2, '서울특별시 마포구 상암동 1618 휴먼시아 상가동 102호', '플로라바움', '010-9188-2728', 'ETC', '37.574948', '126.897842'),
	(3, '서울특별시 마포구 성암로 179', '스타벅스 디지털미디어시티역점', '1522-3232', 'CE7', '37.576124', '126.898930'),
	(4, '서울특별시 마포구 중동 390', 'DMC마포구청아파트', '', 'AD5', '37.573952', '126.901758'),
	(5, '서울측별시 마포구 상암동 1623', '스타벅스 디지털미디어시티역점', '1522-3232', 'CE7', '37.576124395186376', '126.89894222717868'),
	(6, '서울특별시 마포구 상암동 1618 휴먼시아 상가동 102호', '플로라바움', '010-9188-2728', 'ETC', '37.574948', '126.897842'),
	(7, '서울특별시 마포구 중동 390', 'DMC', '02-305-3111', 'AG2', '37.577689', '126.892129'),
	(8, '서울 마포구 상암동 1596', '행복커피', '02-308-7984', 'CE7', '37.582011495162206', '126.88994707387491'),
	(9, '서울 마포구 상암동 1683', '셰프차이나', '02-372-7900', 'FD6', '37.5809249336608', '126.88417050346895'),
	(10, '서울 마포구 상암동 1689', '삼성어린이집', '02-307-7101', 'PS3', '37.579406885912356', '126.88830023397516'),
	(12, '서울 마포구 상암동 1601', '류차이나', '02-307-5077', 'FD6', '37.5800671423264', '126.888913235061'),
	(13, '서울 마포구 상암동 1660', '상암월드컵파크7단지아파트', '', 'AD5', '37.58186636823441', '126.88234419151644'),
	(14, '경남 합천군 합천읍 영창리 898', '황강신소양체육공원', '', 'PO3', '35.5807446260855', '128.170195806889'),
	(15, '경남 합천군 대병면 회양리 700-30', '로우풀', '', 'CE7', '35.525318237179164', '128.0188696210686'),
	(16, '서울 영등포구 여의도동 23-2', '신한투자증권 본사', '1588-0365', 'BK9', '37.5246656750352', '126.924186920328'),
	(17, '서울 강남구 역삼동 718-5', '멀티캠퍼스 역삼', '1544-9001', 'AC5', '37.5012860931305', '127.039604663862');

-- 테이블 neplan.plan 구조 내보내기
CREATE TABLE IF NOT EXISTS `plan` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `dep_datetime` varchar(255) DEFAULT NULL,
  `is_public` bit(1) NOT NULL,
  `title` varchar(255) NOT NULL,
  `file_id` bigint(20) DEFAULT NULL,
  `user_id` bigint(20) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FK1o2qm6b7ugc1pabub41cox7e2` (`file_id`),
  KEY `FK271ok4ss5pcte25w6o3hvv60x` (`user_id`),
  CONSTRAINT `FK1o2qm6b7ugc1pabub41cox7e2` FOREIGN KEY (`file_id`) REFERENCES `file` (`id`),
  CONSTRAINT `FK271ok4ss5pcte25w6o3hvv60x` FOREIGN KEY (`user_id`) REFERENCES `user` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- 테이블 데이터 neplan.plan:~3 rows (대략적) 내보내기
INSERT INTO `plan` (`id`, `dep_datetime`, `is_public`, `title`, `file_id`, `user_id`) VALUES
	(1, '202310060012', b'1', '상암동네 한 바퀴', 27, 1),
	(2, '202310060912', b'1', '상암 맛집 투어', 28, 1),
	(3, '202310062000', b'1', '우리 서울 구경 가자', 28, 1);

-- 테이블 neplan.plan_place 구조 내보내기
CREATE TABLE IF NOT EXISTS `plan_place` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `place_id` bigint(20) DEFAULT NULL,
  `plan_id` bigint(20) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `FKjepuxgcrstbb7ip1jch8fiqd` (`place_id`),
  KEY `FKorxa7r3be86d7yx7dksssiaoi` (`plan_id`),
  CONSTRAINT `FKjepuxgcrstbb7ip1jch8fiqd` FOREIGN KEY (`place_id`) REFERENCES `place` (`id`),
  CONSTRAINT `FKorxa7r3be86d7yx7dksssiaoi` FOREIGN KEY (`plan_id`) REFERENCES `plan` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=90 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- 테이블 데이터 neplan.plan_place:~7 rows (대략적) 내보내기
INSERT INTO `plan_place` (`id`, `place_id`, `plan_id`) VALUES
	(1, 4, 2),
	(2, 2, 2),
	(3, 3, 2),
	(4, 4, 2),
	(5, 5, 2),
	(6, 5, 2),
	(10, 12, 3),
	(87, 8, 1),
	(88, 9, 1),
	(89, 10, 1);

-- 테이블 neplan.user 구조 내보내기
CREATE TABLE IF NOT EXISTS `user` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `address` varchar(512) DEFAULT NULL,
  `email` varchar(255) DEFAULT NULL,
  `name` varchar(255) DEFAULT NULL,
  `password` varchar(255) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=3 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;

-- 테이블 데이터 neplan.user:~2 rows (대략적) 내보내기
INSERT INTO `user` (`id`, `address`, `email`, `name`, `password`) VALUES
	(1, '서울 동대문구 망우로14가길 90', 'wlskb@naver.com', '유지나', 'ginajjang'),
	(2, '서울 동대문구 망우로18가길 106', 'bibibig@naver.com', '김비비', 'BIBIGO');

/*!40103 SET TIME_ZONE=IFNULL(@OLD_TIME_ZONE, 'system') */;
/*!40101 SET SQL_MODE=IFNULL(@OLD_SQL_MODE, '') */;
/*!40014 SET FOREIGN_KEY_CHECKS=IFNULL(@OLD_FOREIGN_KEY_CHECKS, 1) */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40111 SET SQL_NOTES=IFNULL(@OLD_SQL_NOTES, 1) */;
